#include "varroa_stage.h"
#include "src/sd/sd_core.h"
#include "src/ui/ui_web.h"
#include "src/util.h"

#include <memory>
#include "img_converters.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "src/ei/ei_signal_shim.h"

static uint32_t count_varroa_detections(const ei_impulse_result_t& res) {
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
  uint32_t n = 0;
  for (uint32_t i = 0; i < res.bounding_boxes_count; ++i) {
    const auto& bb = res.bounding_boxes[i];
    if (bb.value > VAR_THRESH && bb.width > 0 && bb.height > 0) n++;
  }
  return n;
#else
  (void)res;
  return 0;
#endif
}

static void draw_varroa_boxes(uint8_t* img, int W, int H, const ei_impulse_result_t& res) {
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
  auto clamp_i = [&](int v, int lo, int hi){ return v < lo ? lo : (v > hi ? hi : v); };
  auto put_px=[&](int x,int y,uint8_t r,uint8_t g,uint8_t b){
    if ((unsigned)x<(unsigned)W && (unsigned)y<(unsigned)H){
      size_t i=((size_t)y*W+x)*3; img[i]=r; img[i+1]=g; img[i+2]=b;
    }
  };

  for (uint32_t k=0; k<res.bounding_boxes_count; k++) {
    auto &bb = res.bounding_boxes[k];
    if (bb.value <= VAR_THRESH) continue;

    float cx = bb.x + bb.width  * 0.5f;
    float cy = bb.y + bb.height * 0.5f;

    int x0 = clamp_i((int)lrintf(cx - bb.width  * 0.5f), 0, W-1);
    int y0 = clamp_i((int)lrintf(cy - bb.height * 0.5f), 0, H-1);
    int x1 = clamp_i((int)lrintf(cx + bb.width  * 0.5f), 0, W-1);
    int y1 = clamp_i((int)lrintf(cy + bb.height * 0.5f), 0, H-1);

    uint8_t r=(uint8_t)(bb.value*255), g=0, b=(uint8_t)((1.0f-bb.value)*255);
    for (int x=x0;x<=x1;x++){ put_px(x,y0,r,g,b); put_px(x,y1,r,g,b); }
    for (int y=y0;y<=y1;y++){ put_px(x0,y,r,g,b); put_px(x1,y,r,g,b); }
  }
#endif
}

static uint32_t run_varroa_on_one_crop_and_count(const char* crop_path) {
  if (!g_sd_ok || !crop_path) return 0;

  File f = SD_MMC.open(crop_path, FILE_READ);
  if (!f) { sdlog_printf("VARROA fail open crop=%s\n", crop_path); return 0; }

  size_t sz = f.size();
  std::unique_ptr<uint8_t[]> jpg(new uint8_t[sz]);
  if (!jpg) { f.close(); sdlog_printf("VARROA oom jpg sz=%lu crop=%s\n", (unsigned long)sz, crop_path); return 0; }

  size_t r = f.read(jpg.get(), sz);
  f.close();
  if (r != sz) { sdlog_printf("VARROA read mismatch crop=%s got=%lu expected=%lu\n", crop_path, (unsigned long)r, (unsigned long)sz); return 0; }

  int sw=0, sh=0;
  if (!jpeg_get_dims_v(jpg.get(), sz, sw, sh)) {
    sdlog_printf("VARROA dims fail crop=%s\n", crop_path);
    return 0;
  }

  uint8_t* src_rgb = nullptr;
  bool decoded_ok = false;

  if (sw == CROP_SIZE && sh == CROP_SIZE) {
    decoded_ok = fmt2rgb888(jpg.get(), sz, PIXFORMAT_JPEG, g_crop_rgb);
    src_rgb = g_crop_rgb;
  } else {
    size_t need = (size_t)sw * sh * 3;
    src_rgb = (uint8_t*)ps_malloc(need);
    if (!src_rgb) src_rgb = (uint8_t*)malloc(need);
    if (!src_rgb) { sdlog_printf("VARROA oom src_rgb need=%lu crop=%s\n", (unsigned long)need, crop_path); return 0; }
    decoded_ok = fmt2rgb888(jpg.get(), sz, PIXFORMAT_JPEG, src_rgb);
  }

  if (!decoded_ok) {
    if (src_rgb && src_rgb != g_crop_rgb) free(src_rgb);
    sdlog_printf("VARROA decode fail crop=%s\n", crop_path);
    return 0;
  }

  ei::image::processing::crop_and_interpolate_rgb888(
    src_rgb, sw, sh,
    g_var_snapshot_buf, EI_VARROA_INPUT_WIDTH, EI_VARROA_INPUT_HEIGHT
  );

  if (src_rgb && src_rgb != g_crop_rgb) free(src_rgb);

  ei::signal_t signal;
  signal.total_length = EI_VARROA_INPUT_WIDTH * EI_VARROA_INPUT_HEIGHT;
  signal.get_data = &ei_varroa_get_data;

  ei_impulse_result_t res = {0};
  EI_IMPULSE_ERROR err = process_impulse(&ei_varroa_impulse(), &signal, &res, debug_nn);
  if (err != EI_IMPULSE_OK) {
    sdlog_printf("VARROA process_impulse error=%d crop=%s\n", err, crop_path);
    return 0;
  }

  const uint32_t mites = count_varroa_detections(res);
  if (mites == 0) {
    sdlog_printf("VARROA none (>%.2f) crop=%s\n", VAR_THRESH, crop_path);

    char base_ext[64];
    basename_with_ext_v(crop_path, base_ext, sizeof(base_ext));

    char dst_crop[192];
    snprintf(dst_crop, sizeof(dst_crop), "%s/%s", g_overlays_nomite_dir, base_ext);

    if (sd_writes_enabled()) {
      if (!SD_MMC.exists(dst_crop)) {
        if (sd_copy_file(crop_path, dst_crop)) sdlog_printf("SAVE_OK varroa_no_mite_crop dst=%s src=%s\n", dst_crop, crop_path);
        else                                   sdlog_printf("SAVE_FAIL varroa_no_mite_crop dst=%s src=%s\n", dst_crop, crop_path);
      } else {
        sdlog_printf("SKIP varroa_no_mite_crop exists=%s\n", dst_crop);
      }
    }
    return 0;
  }

  memcpy(g_var_overlay_buf, g_var_snapshot_buf, (size_t)EI_VARROA_INPUT_WIDTH * EI_VARROA_INPUT_HEIGHT * 3);
  draw_varroa_boxes(g_var_overlay_buf, EI_VARROA_INPUT_WIDTH, EI_VARROA_INPUT_HEIGHT, res);

  char base[48];
  basename_no_ext_v(crop_path, base, sizeof(base));

  if (!sd_writes_enabled()) return mites;

  char out_path[128];
  snprintf(out_path, sizeof(out_path), "%s/%s_overlay.jpg", g_overlays_mite_dir, base);

  const size_t pixels = (size_t)EI_VARROA_INPUT_WIDTH * (size_t)EI_VARROA_INPUT_HEIGHT;
  bgr_to_rgb_inplace(g_var_overlay_buf, pixels);
  const bool ok = sd_write_jpg_rgb888(out_path, g_var_overlay_buf, EI_VARROA_INPUT_WIDTH, EI_VARROA_INPUT_HEIGHT, JPEG_QUALITY);
  bgr_to_rgb_inplace(g_var_overlay_buf, pixels);

  if (!ok) sdlog_printf("SAVE_FAIL varroa_overlay path=%s crop=%s\n", out_path, crop_path);
  else     sdlog_printf("SAVE_OK varroa_overlay path=%s crop=%s mites=%lu\n", out_path, crop_path, (unsigned long)mites);

  return mites;
}

uint32_t varroa_run_on_new_crops_and_count() {
  uint32_t mites_total = 0;
  sdlog_printf("VARROA batch start crops=%lu\n", (unsigned long)g_crop_count);

  for (uint32_t i = 0; i < g_crop_count; ++i) {
    web_pump();
    if (should_abort()) break;
    mites_total += run_varroa_on_one_crop_and_count(g_crop_meta[i].path);
    yield();
  }

  sdlog_printf("VARROA batch done mites_total=%lu\n", (unsigned long)mites_total);
  return mites_total;
}
