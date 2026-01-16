#include "bee_stage.h"
#include "src/sd/sd_core.h"
#include "src/util.h"

static void draw_center_boxes(uint8_t* img, int W, int H, const ei_impulse_result_t& res) {
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
  static constexpr int kBoxSize = 4;
  static constexpr int kHalf    = kBoxSize / 2;

  auto clamp_i = [&](int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); };
  auto put_px = [&](int x, int y, uint8_t r, uint8_t g, uint8_t b) {
    if ((unsigned)x < (unsigned)W && (unsigned)y < (unsigned)H) {
      size_t i = ((size_t)y * (size_t)W + (size_t)x) * 3;
      img[i + 0] = r; img[i + 1] = g; img[i + 2] = b;
    }
  };

  for (uint32_t k = 0; k < res.bounding_boxes_count; k++) {
    auto &bb = res.bounding_boxes[k];
    if (bb.value < UI_THRESH) continue;

    const int cx = (int)lrintf(bb.x + bb.width  * 0.5f);
    const int cy = (int)lrintf(bb.y + bb.height * 0.5f);

    int x0 = clamp_i(cx - kHalf, 0, W - 1);
    int y0 = clamp_i(cy - kHalf, 0, H - 1);
    int x1 = clamp_i(x0 + (kBoxSize - 1), 0, W - 1);
    int y1 = clamp_i(y0 + (kBoxSize - 1), 0, W - 1);

    uint8_t r = (uint8_t)(bb.value * 255);
    uint8_t g = 0;
    uint8_t b = (uint8_t)((1.0f - bb.value) * 255);

    for (int x = x0; x <= x1; x++) { put_px(x, y0, r, g, b); put_px(x, y1, r, g, b); }
    for (int y = y0; y <= y1; y++) { put_px(x0, y, r, g, b); put_px(x1, y, r, g, b); }
  }
#endif
}

uint32_t bee_count_detections(const ei_impulse_result_t& res) {
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
  uint32_t n = 0;
  for (uint32_t i = 0; i < res.bounding_boxes_count; ++i) {
    const auto& bb = res.bounding_boxes[i];
    if (bb.value >= UI_THRESH && bb.width > 0 && bb.height > 0) n++;
  }
  return n;
#else
  (void)res;
  return 0;
#endif
}

void bee_log_detections(const ei_impulse_result_t& res) {
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
  sdlog_printf("BEE_DETECTIONS count=%lu (>=%.2f)\n", (unsigned long)res.bounding_boxes_count, UI_THRESH);
  for (uint32_t i = 0; i < res.bounding_boxes_count; ++i) {
    const auto& bb = res.bounding_boxes[i];
    if (bb.value < UI_THRESH) continue;
    sdlog_printf("  bee_bb[%lu] label=%s score=%.3f x=%.1f y=%.1f w=%.1f h=%.1f\n",
                 (unsigned long)i, bb.label, bb.value,
                 (double)bb.x, (double)bb.y, (double)bb.width, (double)bb.height);
  }
#else
  (void)res;
#endif
}

void bee_save_overlay(const ei_impulse_result_t& res) {
  if (!sd_writes_enabled() || !snapshot_buf || !g_bee_overlay_buf) return;

  const int W = EI_CLASSIFIER_INPUT_WIDTH;
  const int H = EI_CLASSIFIER_INPUT_HEIGHT;

  memcpy(g_bee_overlay_buf, snapshot_buf, (size_t)W * H * 3);
  draw_center_boxes(g_bee_overlay_buf, W, H, res);

  char out_path[160];
  snprintf(out_path, sizeof(out_path), "%s/%06lu.jpg",
           g_bee_overlays_dir, (unsigned long)g_frame_counter);

  const size_t pixels = (size_t)W * (size_t)H;
  bgr_to_rgb_inplace(g_bee_overlay_buf, pixels);
  const bool ok = sd_write_jpg_rgb888(out_path, g_bee_overlay_buf, W, H, JPEG_QUALITY);
  bgr_to_rgb_inplace(g_bee_overlay_buf, pixels);

  if (!ok) sdlog_printf("SAVE_FAIL bee_overlay path=%s\n", out_path);
  else     sdlog_printf("SAVE_OK bee_overlay path=%s W=%d H=%d\n", out_path, W, H);
}

bool bee_write_centers_txt(const ei_impulse_result_t& res) {
  if (!sd_writes_enabled()) return false;

  File f = SD_MMC.open(g_last_meta_path, FILE_WRITE);
  if (!f) { sdlog_printf("SAVE_FAIL centers_txt path=%s\n", g_last_meta_path); return false; }

  uint32_t wrote = 0;
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
  for (uint32_t i=0; i<res.bounding_boxes_count; ++i) {
    auto &bb = res.bounding_boxes[i];
    if (bb.value < UI_THRESH) continue;
    float cx = bb.x + bb.width*0.5f;
    float cy = bb.y + bb.height*0.5f;
    char lab[12]; sanitize_label(bb.label, lab);
    f.printf("%u %.3f %.3f %.3f %s\n", (unsigned)i, cx, cy, bb.value, lab);
    wrote++;
  }
#endif
  f.flush();
  f.close();

  if (wrote == 0) {
    sdlog_printf("SAVE_OK centers_txt path=%s entries=0\n", g_last_meta_path);
    return false;
  }

  sdlog_printf("SAVE_OK centers_txt path=%s entries=%lu\n", g_last_meta_path, (unsigned long)wrote);
  return true;
}
