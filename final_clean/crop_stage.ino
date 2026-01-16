#include "crop_stage.h"
#include "src/sd/sd_core.h"
#include "src/ui/ui_web.h"
#include "src/util.h"

#include <memory>
#include "img_converters.h"

void crops_reset() { g_crop_count = 0; }

static uint32_t read_centers_txt(uint32_t idx[], float cx[], float cy[], float sc[],
                                 char lab[][12], uint32_t maxn) {
  if (!g_sd_ok) return 0;

  File f = SD_MMC.open(g_last_meta_path, FILE_READ);
  if (!f) return 0;

  uint32_t n=0;
  while (f.available() && n<maxn) {
    String line = f.readStringUntil('\n');
    if (line.length()<3) continue;

    unsigned ii=0;
    float x=0,y=0,s=0;
    char lbuf[16]={0};

    int m = sscanf(line.c_str(), "%u %f %f %f %15s", &ii, &x, &y, &s, lbuf);
    if (m==5) {
      idx[n]=ii; cx[n]=x; cy[n]=y; sc[n]=s;
      memset(lab[n],0,12);
      strncpy(lab[n], lbuf, 11);
      n++;
    }
  }
  f.close();
  return n;
}

void crops_save_from_last_frame() {
  if (!sd_writes_enabled()) { sdlog_printf("CROPS skip (saving disabled)\n"); crops_reset(); return; }
  crops_reset();

  uint32_t idx[MAX_CROPS]; float cx[MAX_CROPS], cy[MAX_CROPS], sc[MAX_CROPS]; char lab[MAX_CROPS][12];
  uint32_t n = read_centers_txt(idx, cx, cy, sc, lab, MAX_CROPS);
  if (n == 0) { sdlog_printf("CROPS skip (no centers)\n"); return; }

  File jf = SD_MMC.open(g_last_frame_path, FILE_READ);
  if (!jf) { sdlog_printf("CROPS fail open frame path=%s\n", g_last_frame_path); return; }

  size_t sz = jf.size();
  std::unique_ptr<uint8_t[]> jpg(new uint8_t[sz]);
  if (!jpg) { jf.close(); sdlog_printf("CROPS oom jpg sz=%lu\n", (unsigned long)sz); return; }

  size_t r = jf.read(jpg.get(), sz);
  jf.close();
  if (r != sz) { sdlog_printf("CROPS read mismatch got=%lu expected=%lu\n", (unsigned long)r, (unsigned long)sz); return; }

  bool ok = fmt2rgb888(jpg.get(), sz, PIXFORMAT_JPEG, g_fullstage_buf);
  if (!ok) { sdlog_printf("CROPS decode full frame failed\n"); return; }

  int crop_x, crop_y, crop_w, crop_h;
  float scale_x, scale_y;
  ei_calc_crop_map((int)g_full_w, (int)g_full_h,
                   EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT,
                   crop_x, crop_y, crop_w, crop_h, scale_x, scale_y);

  const int half = CROP_SIZE / 2;
  const size_t PIXELS = (size_t)CROP_SIZE * CROP_SIZE;

  sdlog_printf("CROPS start n=%lu CROP_SIZE=%d\n", (unsigned long)n, CROP_SIZE);

  for (uint32_t i = 0; i < n && g_crop_count < MAX_CROPS; ++i) {
    web_pump();
    if (should_abort()) break;

    int cxf = (int)lrintf((float)crop_x + cx[i] * scale_x);
    int cyf = (int)lrintf((float)crop_y + cy[i] * scale_y);

    int x0 = cxf - half;
    int y0 = cyf - half;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x0 > (int)g_full_w - CROP_SIZE) x0 = (int)g_full_w - CROP_SIZE;
    if (y0 > (int)g_full_h - CROP_SIZE) y0 = (int)g_full_h - CROP_SIZE;

    for (int y = 0; y < CROP_SIZE; ++y) {
      const uint8_t* src = g_fullstage_buf + ((size_t)(y0 + y) * g_full_w + x0) * 3;
      memcpy(g_crop_rgb + (size_t)y * CROP_SIZE * 3, src, (size_t)CROP_SIZE * 3);
    }

    bgr_to_rgb_inplace(g_crop_rgb, PIXELS);

    uint8_t* jbuf = nullptr; size_t jlen = 0;
    bool enc = fmt2jpg(g_crop_rgb, PIXELS * 3, CROP_SIZE, CROP_SIZE, PIXFORMAT_RGB888,
                       JPEG_QUALITY, &jbuf, &jlen);
    if (!enc || !jbuf || !jlen) { sdlog_printf("CROPS encode fail i=%lu\n", (unsigned long)i); if (jbuf) free(jbuf); continue; }

    char path[64]; int score_i = (int)lrintf(sc[i] * 100.0f);
    snprintf(path, sizeof(path), "%s/%06lu_%02lu_%s_%u.jpg",
             g_crops_dir,
             (unsigned long)g_frame_counter,
             (unsigned long)g_crop_count,
             lab[i],
             (unsigned)score_i);

    File f = SD_MMC.open(path, FILE_WRITE);
    if (!f) { sdlog_printf("SAVE_FAIL crop_jpg path=%s\n", path); free(jbuf); continue; }

    size_t w = f.write(jbuf, jlen);
    f.close();
    free(jbuf);

    if (w != jlen) {
      sdlog_printf("SAVE_FAIL crop_jpg incomplete path=%s wrote=%lu expected=%lu\n",
                   path, (unsigned long)w, (unsigned long)jlen);
      continue;
    }

    g_crop_meta[g_crop_count].bbox_index = idx[i];
    strncpy(g_crop_meta[g_crop_count].path, path, sizeof(g_crop_meta[g_crop_count].path) - 1);
    g_crop_meta[g_crop_count].path[sizeof(g_crop_meta[g_crop_count].path) - 1] = 0;

    sdlog_printf("SAVE_OK crop_jpg path=%s score=%.3f center=(%.1f,%.1f) full_roi=(%d,%d)\n",
                 path, (double)sc[i], (double)cx[i], (double)cy[i], x0, y0);

    g_crop_count++;
  }

  sdlog_printf("CROPS done saved=%lu\n", (unsigned long)g_crop_count);
}
