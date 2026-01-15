// ================================
// Includes
// ================================
//#include <bee_varroa_merge_inference.h>
#include <merge_b.h>
//#include <merge_inferencing.h>
//#include <merge_bee_real.h>

#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"

#include <memory>
#include <math.h>
#include <stdarg.h>
#include <cstring>

#include <FS.h>
#include <SD_MMC.h>
#include "img_converters.h"

#include "sd_web_ui.h"
#include <Adafruit_NeoPixel.h>

// ================================
// Hardware / Pin Configuration
// ================================
/* ESP32-S3-CAM camera pins */
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5
#define Y2_GPIO_NUM    11
#define Y3_GPIO_NUM    9
#define Y4_GPIO_NUM    8
#define Y5_GPIO_NUM    10
#define Y6_GPIO_NUM    12
#define Y7_GPIO_NUM    18
#define Y8_GPIO_NUM    17
#define Y9_GPIO_NUM    16
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

/* SD 1-bit (S3 SDMMC) */
#define SD_CLK_PIN     39
#define SD_CMD_PIN     38
#define SD_DATA0_PIN   40

/* LED (WS2812/NeoPixel) */
#define LED_PIN   21
#define LED_COUNT 1

// ================================
// Image / Model Dimensions
// ================================
#define FULL_W 1280
#define FULL_H 1024
#define EI_CAMERA_FRAME_BYTE_SIZE 3

static constexpr int CROP_SIZE = 160;
static constexpr int MAX_CROPS = 50;

static constexpr int VAR_W = EI_VARROA_INPUT_WIDTH;
static constexpr int VAR_H = EI_VARROA_INPUT_HEIGHT;

// ================================
// Thresholds / Timing
// ================================
static constexpr float UI_THRESH     = 0.35f;
static constexpr float VAR_THRESH    = 0.50f;
static constexpr int   DROPPED_FRAMES = 10;

static uint32_t INFER_PERIOD_MS = 5000;
static uint32_t last_infer_ms = 0;

static constexpr int JPEG_QUALITY = 100;

// =============================================================
// REPORT NOTES — PIPELINE OVERVIEW (2-stage)
// =============================================================
//
// Goal: Detect bees in a full camera frame (Stage 1), then run a second
// model on per-bee crops to detect Varroa mites (Stage 2).
//
// Inputs:
//   - OV2640 camera provides SXGA (1280x1024) JPEG frames.
//   - Stage 1 model (bee detector) expects EI_CLASSIFIER_INPUT_WIDTH/HEIGHT RGB.
//   - Stage 2 model (varroa detector) expects EI_VARROA_INPUT_WIDTH/HEIGHT RGB.
//
// Outputs (SD card artifacts):
//   1) /frames/boot_xxxxxx/NNNNNN.jpg        (raw full frame JPEG)
//   2) /frames/boot_xxxxxx/NNNNNN.txt        (bee centers + scores + labels)
//   3) /bee_overlays/boot_xxxxxx/NNNNNN.jpg  (Stage-1 input + center markers)
//   4) /crops/boot_xxxxxx/*.jpg              (160x160 bee-centered crops)
//   5) /overlays/boot_xxxxxx/mite/*.jpg      (varroa overlays if mites found)
//   6) /overlays/boot_xxxxxx/no_mite/*.jpg   (crops copied when no mites)
//
// Key metrics:
//   - bees_this: number of bee detections in current frame (>= UI_THRESH)
//   - mites_this: number of varroa detections across all crops (> VAR_THRESH)
//   - avg_weighted = 100 * total_mites / total_bees  (boot-level weighted metric)
//
// Control flags (from Web UI):
//   - g_infer_enabled: start/stop inference loop (kept false initially so UI loads)
//   - g_save_enabled : enable/disable SD writes (browse mode vs capture mode)
//
// Alerts / communication:
//   - NeoPixel LED indicates infestation using avg_weighted threshold.
//   - Web UI shows live stats and provides SD browsing of saved images.
//   - SD log records all detections, counters, and per-boot metadata.
// =============================================================


// ================================
// Global State / Buffers
// ================================
static bool is_initialised = false;
static bool debug_nn = false;

static uint8_t *snapshot_buf        = nullptr;  // EI bee input
static uint8_t *g_fullstage_buf     = nullptr;  // full frame decoded RGB888
static uint8_t *g_crop_rgb          = nullptr;  // crop buffer RGB888 (then encoded)
static uint8_t *g_bee_overlay_buf   = nullptr;  // bee overlay (EI input size)

static uint16_t g_full_w = FULL_W, g_full_h = FULL_H;

static char g_last_frame_path[128] = {0};
static char g_last_meta_path[128]  = {0};
static uint32_t g_frame_counter = 0;

static bool g_sd_ok = false;
static bool g_web_started = false;

volatile bool g_infer_enabled = false; // start paused so UI loads reliably
volatile bool g_save_enabled  = false;

static inline bool sd_writes_enabled() { return g_sd_ok && g_save_enabled; }
static inline void web_pump() { if (g_web_started) sd_web_ui_loop(); }
static inline bool should_abort() { return !g_infer_enabled; }

// ================================
// Forward Declarations
// ================================
// SD / FS
static bool ensure_dir(const char* path);
static bool wipe_dir_contents(const char* dir_path);
static bool delete_dir_tree(const char* dir_path);
static void build_child_path(const char* parent, const char* entry_name, char* out, size_t out_sz);
static bool init_sdcard();
static bool init_boot_session_dirs_and_log();
static bool sd_copy_file(const char* src_path, const char* dst_path);

// Logging
static uint32_t read_u32_file_or_default(const char* path, uint32_t def_val);
static bool write_u32_file(const char* path, uint32_t v);
static void sdlog_printf(const char* fmt, ...);
static uint32_t allocate_unique_boot_id();

// Color / encode
static inline void bgr_to_rgb_inplace(uint8_t* buf, size_t pixels);
static bool write_jpg_rgb888_to_sd(const char* out_path, const uint8_t* rgb, int W, int H);

// Bee stage helpers
static uint32_t count_bee_detections(const ei_impulse_result_t& res);
static void log_bee_detections(const ei_impulse_result_t& res);
static void draw_center_boxes(uint8_t* img, int W, int H, const ei_impulse_result_t& res);
static void save_bee_overlay_to_sd(const ei_impulse_result_t& res);
static bool save_fb_jpeg_to_sd(camera_fb_t* fb);
static void sanitize_label(const char* in, char out[12]);
static bool write_centers_txt(const ei_impulse_result_t& res);
static uint32_t read_centers_txt(uint32_t idx[], float cx[], float cy[], float sc[], char lab[][12], uint32_t maxn);
static void ei_calc_crop_map(int src_w, int src_h, int dst_w, int dst_h,
                             int &crop_x, int &crop_y, int &crop_w, int &crop_h,
                             float &scale_x, float &scale_y);
static void save_crops();

// Varroa stage helpers
static bool jpeg_get_dims_v(const uint8_t* data, size_t len, int& w, int& h);
static int varroa_get_data_packed(size_t offset, size_t length, float *out_ptr);
static uint32_t count_varroa_detections(const ei_impulse_result_t& res);
static void log_varroa_detections(const char* crop_path, const ei_impulse_result_t& res);
static void draw_varroa_boxes(uint8_t* img, int W, int H, const ei_impulse_result_t& res);
static void basename_no_ext_v(const char* path, char* out, size_t out_sz);
static void basename_with_ext_v(const char* path, char* out, size_t out_sz);
static uint32_t run_varroa_on_one_crop_and_count(const char* crop_path);
static uint32_t run_varroa_on_new_crops_and_count();

// Camera glue + main loop
bool ei_camera_init(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);
static void run_inference_once();

// ================================
// Logging + Boot Session Dirs
// ================================
static char g_frames_dir[64]        = {0};
static char g_bee_overlays_dir[64]  = {0};
static char g_crops_dir[64]         = {0};
static char g_overlays_dir[64]      = {0};
static char g_overlays_mite_dir[96]   = {0};
static char g_overlays_nomite_dir[96] = {0};

static constexpr bool LOG_ECHO_SERIAL = true;

static const char* LOG_DIR = "/logs";
static const char* BOOT_ID_PATH = "/logs/boot_id.txt";
static constexpr const char* OVERLAY_MITE_SUBDIR    = "mite";
static constexpr const char* OVERLAY_NO_MITE_SUBDIR = "no_mite";

static constexpr size_t SD_COPY_BUF_SIZE = 1024;
static uint8_t g_sd_copy_buf[SD_COPY_BUF_SIZE];

static uint32_t g_boot_id = 0;
static char g_log_path[128] = {0};
static File g_log_file;

static uint32_t read_u32_file_or_default(const char* path, uint32_t def_val) {
  File f = SD_MMC.open(path, FILE_READ);
  if (!f) return def_val;
  String s = f.readStringUntil('\n');
  f.close();
  s.trim();
  if (!s.length()) return def_val;
  return (uint32_t)strtoul(s.c_str(), nullptr, 10);
}

static bool write_u32_file(const char* path, uint32_t v) {
  File f = SD_MMC.open(path, FILE_WRITE);
  if (!f) return false;
  f.printf("%lu\n", (unsigned long)v);
  f.flush();
  f.close();
  return true;
}

// =============================================================
// COMMUNICATION — SD logging (audit trail / traceability)
// =============================================================
// sdlog_printf() writes structured logs to both Serial (optional) and SD log file.
// This is used to record:
//   - Per-frame inference events (cycle start, capture success/failure)
//   - Stage 1 detections and bounding boxes
//   - Crop generation status and paths
//   - Stage 2 detections and overlay/copy results
//   - Counters, round summaries, and avg_weighted
//
// This provides reproducibility and offline analysis after deployment.
// =============================================================


static void sdlog_printf(const char* fmt, ...) {
  char buf[384];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (LOG_ECHO_SERIAL) Serial.print(buf);

  if (!g_sd_ok || !g_save_enabled) return;
  if (!g_log_file) return;

  g_log_file.print(buf);
  g_log_file.flush();
}

static uint32_t allocate_unique_boot_id() {
  uint32_t cand = read_u32_file_or_default(BOOT_ID_PATH, 0) + 1;

  if (!SD_MMC.exists(LOG_DIR)) SD_MMC.mkdir(LOG_DIR);

  while (true) {
    char test[128];
    snprintf(test, sizeof(test), "%s/boot_%06lu.txt", LOG_DIR, (unsigned long)cand);
    if (!SD_MMC.exists(test)) break;
    cand++;
  }

  (void)write_u32_file(BOOT_ID_PATH, cand);
  return cand;
}

static bool init_boot_session_dirs_and_log() {
  if (!g_sd_ok) return false;

  if (!ensure_dir("/frames")) return false;
  if (!ensure_dir("/bee_overlays")) return false;
  if (!ensure_dir("/crops")) return false;
  if (!ensure_dir("/overlays")) return false;
  if (!ensure_dir(LOG_DIR)) return false;

  if (!wipe_dir_contents("/frames")) return false;
  if (!wipe_dir_contents("/crops")) return false;

  g_boot_id = allocate_unique_boot_id();

  snprintf(g_frames_dir,       sizeof(g_frames_dir),       "/frames/boot_%06lu",       (unsigned long)g_boot_id);
  snprintf(g_bee_overlays_dir, sizeof(g_bee_overlays_dir), "/bee_overlays/boot_%06lu", (unsigned long)g_boot_id);
  snprintf(g_crops_dir,        sizeof(g_crops_dir),        "/crops/boot_%06lu",        (unsigned long)g_boot_id);
  snprintf(g_overlays_dir,     sizeof(g_overlays_dir),     "/overlays/boot_%06lu",     (unsigned long)g_boot_id);

  if (!ensure_dir(g_frames_dir)) return false;
  if (!ensure_dir(g_bee_overlays_dir)) return false;
  if (!ensure_dir(g_crops_dir)) return false;
  if (!ensure_dir(g_overlays_dir)) return false;

  snprintf(g_overlays_mite_dir, sizeof(g_overlays_mite_dir), "%s/%s", g_overlays_dir, OVERLAY_MITE_SUBDIR);
  snprintf(g_overlays_nomite_dir, sizeof(g_overlays_nomite_dir), "%s/%s", g_overlays_dir, OVERLAY_NO_MITE_SUBDIR);

  if (!ensure_dir(g_overlays_mite_dir)) return false;
  if (!ensure_dir(g_overlays_nomite_dir)) return false;

  snprintf(g_log_path, sizeof(g_log_path), "%s/boot_%06lu.txt", LOG_DIR, (unsigned long)g_boot_id);
  g_log_file = SD_MMC.open(g_log_path, FILE_WRITE);
  if (!g_log_file) return false;

  g_log_file.printf("=== BOOT %lu === millis=%lu ===\n", (unsigned long)g_boot_id, (unsigned long)millis());
  g_log_file.printf("DIR frames=%s\nDIR bee_overlays=%s\nDIR crops=%s\nDIR overlays=%s\n",
                    g_frames_dir, g_bee_overlays_dir, g_crops_dir, g_overlays_dir);
  g_log_file.printf("DIR overlays/mite=%s\nDIR overlays/no_mite=%s\n",
                    g_overlays_mite_dir, g_overlays_nomite_dir);
  g_log_file.flush();

  return true;
}

// ================================
// Counting (bee/mite rounds)
// ================================
static constexpr uint32_t TARGET_BEES_PER_ROUND = 100;

static uint32_t g_round_bees  = 0;
static uint32_t g_round_mites = 0;

volatile uint32_t g_total_bees  = 0;
volatile uint32_t g_total_mites = 0;

static uint32_t count_bee_detections(const ei_impulse_result_t& res) {
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

static void maybe_finalize_round_and_log() {
  if (g_round_bees < TARGET_BEES_PER_ROUND) return;

  const double round_pct = (g_round_bees > 0)
    ? (100.0 * (double)g_round_mites / (double)g_round_bees)
    : 0.0;

  const double avg_weighted = (g_total_bees > 0)
    ? (100.0 * (double)g_total_mites / (double)g_total_bees)
    : 0.0;

  sdlog_printf(
    "[ROUND DONE] bees=%lu mites=%lu => %.2f%% | avg_weighted=%.2f%% | totals bees=%lu mites=%lu\n",
    (unsigned long)g_round_bees,
    (unsigned long)g_round_mites,
    round_pct,
    avg_weighted,
    (unsigned long)g_total_bees,
    (unsigned long)g_total_mites
  );

  g_round_bees = 0;
  g_round_mites = 0;
}

// ================================
// LED (avg_weighted indicator)
// ================================
static constexpr double  LED_INFEST_THRESH_PCT = 10.0;  // avg_weighted threshold
static constexpr uint8_t LED_BRIGHTNESS = 50;           // 0-255

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

enum class LedState : uint8_t { Unknown, Green, Red };
static LedState g_led_state = LedState::Unknown;

static inline double avg_weighted_pct_boot() {
  if (g_total_bees == 0) return 0.0;
  return 100.0 * (double)g_total_mites / (double)g_total_bees;
}

static void led_set_rgb(uint8_t r, uint8_t g, uint8_t b) {
  strip.setPixelColor(0, strip.Color(r, g, b));
  strip.show();
}

// =============================================================
// ALERT MECHANISM — NeoPixel status indicator
// =============================================================
// The device provides a simple on-hardware alert:
//   - Compute avg_weighted = 100 * total_mites / total_bees
//   - If avg_weighted > LED_INFEST_THRESH_PCT  => LED = RED  (high infestation)
//   - Else                                     => LED = GREEN (normal)
//
// This allows operation without a phone/PC UI in the field.
// =============================================================


static void led_update_from_avg_weighted(bool force = false) {
  const double avg_w = avg_weighted_pct_boot();
  const LedState target = (avg_w > LED_INFEST_THRESH_PCT) ? LedState::Red : LedState::Green;

  if (!force && target == g_led_state) return;
  g_led_state = target;

  if (target == LedState::Red) led_set_rgb(255, 0, 0);
  else                        led_set_rgb(0, 255, 0);
}

// ================================
// Camera configuration
// ================================
static camera_config_t camera_config = {
  .pin_pwdn = PWDN_GPIO_NUM,
  .pin_reset = RESET_GPIO_NUM,
  .pin_xclk = XCLK_GPIO_NUM,
  .pin_sscb_sda = SIOD_GPIO_NUM,
  .pin_sscb_scl = SIOC_GPIO_NUM,
  .pin_d7 = Y9_GPIO_NUM,
  .pin_d6 = Y8_GPIO_NUM,
  .pin_d5 = Y7_GPIO_NUM,
  .pin_d4 = Y6_GPIO_NUM,
  .pin_d3 = Y5_GPIO_NUM,
  .pin_d2 = Y4_GPIO_NUM,
  .pin_d1 = Y3_GPIO_NUM,
  .pin_d0 = Y2_GPIO_NUM,
  .pin_vsync = VSYNC_GPIO_NUM,
  .pin_href = HREF_GPIO_NUM,
  .pin_pclk = PCLK_GPIO_NUM,
  .xclk_freq_hz = 20000000,
  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,
  .pixel_format = PIXFORMAT_JPEG,
  .frame_size = FRAMESIZE_SXGA,
  .jpeg_quality = 10,
  .fb_count = 1,
  .fb_location = CAMERA_FB_IN_PSRAM,
  .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

// ================================
// SD / FS Utilities
// ================================
static bool ensure_dir(const char* path) {
  if (!SD_MMC.exists(path)) {
    if (!SD_MMC.mkdir(path)) {
      Serial.printf("mkdir %s failed\n", path);
      return false;
    }
  }
  return true;
}

static void build_child_path(const char* parent, const char* entry_name, char* out, size_t out_sz) {
  if (!out || out_sz == 0) return;
  out[0] = 0;
  if (!parent || !parent[0] || !entry_name || !entry_name[0]) return;

  if (entry_name[0] == '/') snprintf(out, out_sz, "%s", entry_name);
  else                      snprintf(out, out_sz, "%s/%s", parent, entry_name);
}

static bool delete_dir_tree(const char* dir_path) {
  if (!dir_path || !dir_path[0]) return false;
  if (!wipe_dir_contents(dir_path)) return false;
  return SD_MMC.rmdir(dir_path);
}

static bool wipe_dir_contents(const char* dir_path) {
  if (!g_sd_ok || !dir_path || !dir_path[0]) return false;

  File dir = SD_MMC.open(dir_path);
  if (!dir) return false;
  if (!dir.isDirectory()) { dir.close(); return false; }

  File entry = dir.openNextFile();
  while (entry) {
    const bool is_dir = entry.isDirectory();
    char child[192];
    build_child_path(dir_path, entry.name(), child, sizeof(child));
    entry.close();

    if (!child[0]) { entry = dir.openNextFile(); continue; }

    if (is_dir) {
      if (!delete_dir_tree(child)) { dir.close(); return false; }
    } else {
      if (!SD_MMC.remove(child)) { dir.close(); return false; }
    }

    entry = dir.openNextFile();
  }

  dir.close();
  return true;
}

static bool init_sdcard() {
  SD_MMC.setPins(SD_CLK_PIN, SD_CMD_PIN, SD_DATA0_PIN);
  if (!SD_MMC.begin("/sdcard", true, false)) {
    Serial.println("SD_MMC.begin() failed.");
    return false;
  }
  Serial.println("SD card mounted: /sdcard");
  return true;
}

static bool sd_copy_file(const char* src_path, const char* dst_path) {
  if (!sd_writes_enabled() || !src_path || !dst_path) return false;

  File src = SD_MMC.open(src_path, FILE_READ);
  if (!src) return false;

  File dst = SD_MMC.open(dst_path, FILE_WRITE);
  if (!dst) { src.close(); return false; }

  while (true) {
    size_t r = src.read(g_sd_copy_buf, SD_COPY_BUF_SIZE);
    if (r == 0) break;
    size_t w = dst.write(g_sd_copy_buf, r);
    if (w != r) { src.close(); dst.close(); return false; }
  }

  dst.flush();
  src.close();
  dst.close();
  return true;
}

// ================================
// Color / JPEG helpers
// ================================
static inline void bgr_to_rgb_inplace(uint8_t* buf, size_t pixels) {
  for (size_t i = 0; i < pixels; ++i) {
    uint8_t* p = &buf[i * 3];
    uint8_t t = p[0];
    p[0] = p[2];
    p[2] = t;
  }
}

static bool write_jpg_rgb888_to_sd(const char* out_path, const uint8_t* rgb, int W, int H) {
  if (!sd_writes_enabled() || !out_path || !rgb || W <= 0 || H <= 0) return false;

  uint8_t* jbuf = nullptr;
  size_t jlen = 0;
  const bool enc = fmt2jpg(
    (uint8_t*)rgb,
    (size_t)W * (size_t)H * 3u,
    (uint16_t)W,
    (uint16_t)H,
    PIXFORMAT_RGB888,
    (uint8_t)JPEG_QUALITY,
    &jbuf,
    &jlen
  );
  if (!enc || !jbuf || !jlen) { if (jbuf) free(jbuf); return false; }

  File f = SD_MMC.open(out_path, FILE_WRITE);
  if (!f) { free(jbuf); return false; }
  const size_t w = f.write(jbuf, jlen);
  f.flush(); f.close();
  free(jbuf);
  return (w == jlen);
}

// ================================
// Bee overlay drawing / logging
// ================================
static void draw_center_boxes(uint8_t* img, int W, int H, const ei_impulse_result_t& res) {
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
  static constexpr int kBoxSize = 4;
  static constexpr int kHalf    = kBoxSize / 2;

  auto clamp_i = [&](int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); };
  auto put_px = [&](int x, int y, uint8_t r, uint8_t g, uint8_t b) {
    if ((unsigned)x < (unsigned)W && (unsigned)y < (unsigned)H) {
      size_t i = ((size_t)y * (size_t)W + (size_t)x) * 3;
      img[i + 0] = r;
      img[i + 1] = g;
      img[i + 2] = b;
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
    int y1 = clamp_i(y0 + (kBoxSize - 1), 0, H - 1);

    uint8_t r = (uint8_t)(bb.value * 255);
    uint8_t g = 0;
    uint8_t b = (uint8_t)((1.0f - bb.value) * 255);

    for (int x = x0; x <= x1; x++) { put_px(x, y0, r, g, b); put_px(x, y1, r, g, b); }
    for (int y = y0; y <= y1; y++) { put_px(x0, y, r, g, b); put_px(x1, y, r, g, b); }
  }
#endif
}

static void log_bee_detections(const ei_impulse_result_t& res) {
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

static void save_bee_overlay_to_sd(const ei_impulse_result_t& res) {
  if (!sd_writes_enabled() || !snapshot_buf || !g_bee_overlay_buf) return;

  const int W = EI_CLASSIFIER_INPUT_WIDTH;
  const int H = EI_CLASSIFIER_INPUT_HEIGHT;

  memcpy(g_bee_overlay_buf, snapshot_buf, (size_t)W * H * 3);
  draw_center_boxes(g_bee_overlay_buf, W, H, res);

  char out_path[160];
  snprintf(out_path, sizeof(out_path),
           "%s/%06lu.jpg", g_bee_overlays_dir, (unsigned long)g_frame_counter);

  const size_t pixels = (size_t)W * (size_t)H;
  bgr_to_rgb_inplace(g_bee_overlay_buf, pixels);   // match crop JPG pipeline
  const bool ok = write_jpg_rgb888_to_sd(out_path, g_bee_overlay_buf, W, H);
  bgr_to_rgb_inplace(g_bee_overlay_buf, pixels);   // restore

  if (!ok) sdlog_printf("SAVE_FAIL bee_overlay path=%s\n", out_path);
  else     sdlog_printf("SAVE_OK bee_overlay path=%s W=%d H=%d\n", out_path, W, H);
}

// ================================
// Frame + centers meta I/O
// ================================
static bool save_fb_jpeg_to_sd(camera_fb_t* fb) {
  if (!sd_writes_enabled() || !fb || fb->format != PIXFORMAT_JPEG) return false;

  snprintf(g_last_frame_path, sizeof(g_last_frame_path),
           "%s/%06lu.jpg", g_frames_dir, (unsigned long)g_frame_counter);

  File f = SD_MMC.open(g_last_frame_path, FILE_WRITE);
  if (!f) { sdlog_printf("SAVE_FAIL frame_jpeg path=%s\n", g_last_frame_path); return false; }

  size_t w = f.write(fb->buf, fb->len);
  f.close();

  if (w != fb->len) {
    sdlog_printf("SAVE_FAIL frame_jpeg incomplete path=%s wrote=%lu expected=%lu\n",
                 g_last_frame_path, (unsigned long)w, (unsigned long)fb->len);
    return false;
  }

  snprintf(g_last_meta_path, sizeof(g_last_meta_path),
           "%s/%06lu.txt", g_frames_dir, (unsigned long)g_frame_counter);

  sdlog_printf("SAVE_OK frame_jpeg path=%s bytes=%lu\n", g_last_frame_path, (unsigned long)fb->len);
  return true;
}

static void sanitize_label(const char* in, char out[12]) {
  memset(out, 0, 12);
  uint8_t j=0;
  for (const char* p=in; *p && j<11; ++p) {
    char c=*p;
    if ((c>='A'&&c<='Z')||(c>='a'&&c<='z')||(c>='0'&&c<='9')) out[j++]=c;
    else if (c==' '||c=='-'||c=='.') out[j++]='_';
  }
  if (j==0) strncpy(out,"obj",11);
}

static bool write_centers_txt(const ei_impulse_result_t& res) {
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

// ================================
// Crop bookkeeping
// ================================
struct CropMeta { uint32_t bbox_index; char path[128]; };
static CropMeta g_crop_meta[MAX_CROPS];
static uint32_t g_crop_count = 0;

static void reset_crop_meta() { g_crop_count = 0; }

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

static void ei_calc_crop_map(int src_w, int src_h, int dst_w, int dst_h,
                             int &crop_x, int &crop_y, int &crop_w, int &crop_h,
                             float &scale_x, float &scale_y) {
  if (src_w <= 0 || src_h <= 0 || dst_w <= 0 || dst_h <= 0) {
    crop_x = 0; crop_y = 0; crop_w = src_w; crop_h = src_h;
    scale_x = (dst_w > 0) ? (float)src_w / (float)dst_w : 1.0f;
    scale_y = (dst_h > 0) ? (float)src_h / (float)dst_h : 1.0f;
    return;
  }

  const float src_ar = (float)src_w / (float)src_h;
  const float dst_ar = (float)dst_w / (float)dst_h;

  if (src_ar > dst_ar) {
    crop_h = src_h;
    crop_w = (int)lrintf((float)src_h * dst_ar);
    if (crop_w < 1) crop_w = 1;
    if (crop_w > src_w) crop_w = src_w;
    crop_x = (src_w - crop_w) / 2;
    crop_y = 0;
  } else {
    crop_w = src_w;
    crop_h = (int)lrintf((float)src_w / dst_ar);
    if (crop_h < 1) crop_h = 1;
    if (crop_h > src_h) crop_h = src_h;
    crop_x = 0;
    crop_y = (src_h - crop_h) / 2;
  }

  scale_x = (float)crop_w / (float)dst_w;
  scale_y = (float)crop_h / (float)dst_h;
}

// =============================================================
// PREPROCESSING (Stage 1 coords -> full-frame crops)
// =============================================================
// Bee detections are produced in the Stage 1 model input coordinate system
// (EI_CLASSIFIER_INPUT_WIDTH x EI_CLASSIFIER_INPUT_HEIGHT).
//
// However, crops are extracted from the *original* full-resolution frame
// (SXGA RGB buffer) to preserve detail for Stage 2 (varroa).
//
// Mapping approach:
//   - ei_calc_crop_map() reconstructs the "center-crop window" EI uses when
//     resizing the full frame to the Stage 1 input. It outputs:
//       crop_x, crop_y, crop_w, crop_h, scale_x, scale_y
//
//   - For each detected bee center (cx,cy) in Stage 1 coordinates:
//       full_x = crop_x + cx * scale_x
//       full_y = crop_y + cy * scale_y
//
// Crop extraction:
//   - Extract a fixed CROP_SIZE x CROP_SIZE patch (default 160x160) around
//     (full_x, full_y), clamped to image boundaries.
//   - Encode crop to JPEG and save under /crops/boot_xxxxxx/
//
// Output bookkeeping:
//   - g_crop_meta[] stores the crop path for downstream Stage 2 processing.
// =============================================================


static void save_crops() {
  if (!sd_writes_enabled()) { sdlog_printf("CROPS skip (saving disabled)\n"); reset_crop_meta(); return; }
  reset_crop_meta();

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
                   crop_x, crop_y, crop_w, crop_h,
                   scale_x, scale_y);

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
    snprintf(path, sizeof(path),
             "%s/%06lu_%02lu_%s_%u.jpg",
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

// ================================
// Varroa stage buffers + helpers
// ================================
static uint8_t *g_var_snapshot_buf = nullptr;
static uint8_t *g_var_overlay_buf  = nullptr;

static bool jpeg_get_dims_v(const uint8_t* data, size_t len, int& w, int& h) {
  w = h = 0;
  if (len < 4 || data[0]!=0xFF || data[1]!=0xD8) return false;
  size_t i = 2;
  while (i + 8 < len) {
    if (data[i] != 0xFF) { i++; continue; }
    uint8_t m = data[i+1];
    if (m == 0xD9 || m == 0xDA) break;
    uint16_t seglen = (i+3 < len) ? ((data[i+2] << 8) | data[i+3]) : 0;
    if (seglen < 2 || i + 2 + seglen > len) break;
    if (m == 0xC0 || m == 0xC2) {
      if (seglen >= 7) {
        h = (data[i+5] << 8) | data[i+6];
        w = (data[i+7] << 8) | data[i+8];
        return (w > 0 && h > 0);
      }
    }
    i += 2 + seglen;
  }
  return false;
}

static int varroa_get_data_packed(size_t offset, size_t length, float *out_ptr) {
  size_t pixel_ix = offset * 3;
  for (size_t i = 0; i < length; i++, pixel_ix += 3) {
    out_ptr[i] = (float)((uint32_t)g_var_snapshot_buf[pixel_ix] << 16 |
                         (uint32_t)g_var_snapshot_buf[pixel_ix + 1] << 8  |
                         (uint32_t)g_var_snapshot_buf[pixel_ix + 2]);
  }
  return 0;
}

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

static void log_varroa_detections(const char* crop_path, const ei_impulse_result_t& res) {
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
  sdlog_printf("VARROA_DETECTIONS crop=%s count=%lu (>%.2f)\n",
               crop_path ? crop_path : "(null)",
               (unsigned long)res.bounding_boxes_count,
               VAR_THRESH);

  for (uint32_t i = 0; i < res.bounding_boxes_count; ++i) {
    const auto& bb = res.bounding_boxes[i];
    if (bb.value <= VAR_THRESH) continue;
    sdlog_printf("  mite_bb[%lu] label=%s score=%.3f x=%.1f y=%.1f w=%.1f h=%.1f\n",
                 (unsigned long)i, bb.label, bb.value,
                 (double)bb.x, (double)bb.y, (double)bb.width, (double)bb.height);
  }
#else
  (void)crop_path; (void)res;
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

static void basename_no_ext_v(const char* path, char* out, size_t out_sz) {
  if (!out || out_sz == 0) return;
  out[0] = 0;

  const char* base = strrchr(path, '/');
  base = base ? (base + 1) : path;

  const char* dot = strrchr(base, '.');
  size_t n = dot ? (size_t)(dot - base) : strlen(base);
  if (n >= out_sz) n = out_sz - 1;
  memcpy(out, base, n);
  out[n] = 0;
}

static void basename_with_ext_v(const char* path, char* out, size_t out_sz) {
  if (!out || out_sz == 0) return;
  out[0] = 0;

  const char* base = strrchr(path, '/');
  base = base ? (base + 1) : path;

  size_t n = strlen(base);
  if (n >= out_sz) n = out_sz - 1;
  memcpy(out, base, n);
  out[n] = 0;
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

// =============================================================
// PREPROCESSING (Stage 2 input generation)
// =============================================================
// Each crop is decoded from JPEG -> RGB, then resized to the varroa model
// input dimensions (VAR_W x VAR_H) using crop_and_interpolate_rgb888().
//
// Result:
//   - g_var_snapshot_buf becomes the Stage 2 model input image.
// =============================================================

  ei::image::processing::crop_and_interpolate_rgb888(
    src_rgb, sw, sh,
    g_var_snapshot_buf, VAR_W, VAR_H
  );

  if (src_rgb && src_rgb != g_crop_rgb) free(src_rgb);

  ei::signal_t signal;
  signal.total_length = VAR_W * VAR_H;
  signal.get_data = &varroa_get_data_packed;

  ei_impulse_result_t res = {0};
  EI_IMPULSE_ERROR err = process_impulse(&ei_varroa_impulse(), &signal, &res, debug_nn);
  if (err != EI_IMPULSE_OK) {
    sdlog_printf("VARROA process_impulse error=%d crop=%s\n", err, crop_path);
    return 0;
  }

  log_varroa_detections(crop_path, res);

  const uint32_t mites = count_varroa_detections(res);
  if (mites == 0) {
    sdlog_printf("VARROA none (>%.2f) crop=%s\n", VAR_THRESH, crop_path);

    char base_ext[64];
    basename_with_ext_v(crop_path, base_ext, sizeof(base_ext));

    char dst_crop[192];
    snprintf(dst_crop, sizeof(dst_crop), "%s/%s", g_overlays_nomite_dir, base_ext);

    if (sd_writes_enabled()) {
      if (!SD_MMC.exists(dst_crop)) {
        if (sd_copy_file(crop_path, dst_crop)) {
          sdlog_printf("SAVE_OK varroa_no_mite_crop dst=%s src=%s\n", dst_crop, crop_path);
        } else {
          sdlog_printf("SAVE_FAIL varroa_no_mite_crop dst=%s src=%s\n", dst_crop, crop_path);
        }
      } else {
        sdlog_printf("SKIP varroa_no_mite_crop exists=%s\n", dst_crop);
      }
    }
    return 0;
  }

  memcpy(g_var_overlay_buf, g_var_snapshot_buf, (size_t)VAR_W * VAR_H * 3);
  draw_varroa_boxes(g_var_overlay_buf, VAR_W, VAR_H, res);

  char base[48];
  basename_no_ext_v(crop_path, base, sizeof(base));

  if (!sd_writes_enabled()) return mites;

  char out_path[96];
  snprintf(out_path, sizeof(out_path), "%s/%s_overlay.jpg", g_overlays_mite_dir, base);

  const size_t pixels = (size_t)VAR_W * (size_t)VAR_H;
  bgr_to_rgb_inplace(g_var_overlay_buf, pixels);   // match crop JPG pipeline
  const bool ok = write_jpg_rgb888_to_sd(out_path, g_var_overlay_buf, VAR_W, VAR_H);
  bgr_to_rgb_inplace(g_var_overlay_buf, pixels);   // restore

  if (!ok) sdlog_printf("SAVE_FAIL varroa_overlay path=%s crop=%s\n", out_path, crop_path);
  else     sdlog_printf("SAVE_OK varroa_overlay path=%s crop=%s mites=%lu\n", out_path, crop_path, (unsigned long)mites);

  return mites;
}

static uint32_t run_varroa_on_new_crops_and_count() {
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

// ================================
// Camera init + capture (EI hooks)
// ================================
bool ei_camera_init(void) {
  if (is_initialised) return true;

  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  for (int i = 0; i < DROPPED_FRAMES; ++i) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    delay(10);
  }

  is_initialised = true;
  return true;
}

// =============================================================
// PREPROCESSING (Stage 1 input generation)
// =============================================================
// Camera returns a JPEG-compressed SXGA frame (1280x1024).
// Steps:
//   1) Acquire JPEG frame: esp_camera_fb_get()
//   2) Optionally archive raw JPEG to SD (/frames/boot_xxxxxx/NNNNNN.jpg)
//   3) Decode JPEG -> full RGB888 buffer (g_fullstage_buf)
//   4) Convert full RGB to the model input size using Edge Impulse utility:
//        crop_and_interpolate_rgb888(fullRGB -> snapshot_buf)
//      This matches EI's expected "center-crop + resize" behavior for inference.
//
// Result:
//   - snapshot_buf contains RGB888 image sized exactly to
//     EI_CLASSIFIER_INPUT_WIDTH x EI_CLASSIFIER_INPUT_HEIGHT,
//     used as the Stage 1 model input.
// =============================================================


bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
  if (!is_initialised) { sdlog_printf("ERR camera not initialized\n"); return false; }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { sdlog_printf("ERR camera capture failed\n"); return false; }

  g_full_w = fb->width;
  g_full_h = fb->height;

  (void)save_fb_jpeg_to_sd(fb);

  bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, g_fullstage_buf);
  esp_camera_fb_return(fb);

  if (!converted) { sdlog_printf("ERR full decode failed\n"); return false; }

  if ((img_width != g_full_w) || (img_height != g_full_h)) {
    ei::image::processing::crop_and_interpolate_rgb888(
      g_fullstage_buf, g_full_w, g_full_h,
      out_buf, img_width, img_height);
  } else {
    memcpy(out_buf, g_fullstage_buf, (size_t)img_width*img_height*3);
  }
  return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  size_t pixel_ix = offset * 3;
  for (size_t i=0;i<length;i++, pixel_ix+=3) {
    out_ptr[i] = (snapshot_buf[pixel_ix] << 16) |
                 (snapshot_buf[pixel_ix + 1] << 8) |
                  snapshot_buf[pixel_ix + 2];
  }
  return 0;
}

// =============================================================
// INFERENCE LOGIC (One full cycle: Bee -> Crops -> Varroa -> Counters)
// =============================================================
// This function executes one timed cycle (triggered every INFER_PERIOD_MS):
//
// 1) Capture + Stage 1 preprocessing:
//    - ei_camera_capture() archives full JPEG + produces snapshot_buf resized for EI.
//
// 2) Stage 1 inference (bee detector):
//    - run_classifier(signal, result)
//    - Filter detections using UI_THRESH
//    - Log detections to SD log
//    - Save a Stage-1 overlay image with center markers (/bee_overlays)
//
// 3) Metadata export (bridge between stages):
//    - Write centers file (/frames/.../NNNNNN.txt): bboxIndex, cx, cy, score, label
//
// 4) Crop generation from full-resolution image:
//    - Re-map Stage 1 center coordinates back to full frame via ei_calc_crop_map()
//    - Extract fixed-size crops (160x160) and save under /crops
//
// 5) Stage 2 inference (varroa detector on each crop):
//    - For each saved crop:
//        decode JPEG -> RGB
//        resize -> (VAR_W x VAR_H)
//        process_impulse(ei_varroa_impulse, ...)
//    - Count varroa detections using VAR_THRESH
//    - If mites found: save overlay under /overlays/.../mite
//      Else: copy crop under /overlays/.../no_mite
//
// 6) Update metrics + alerts:
//    - g_total_bees  += bees_this
//    - g_total_mites += mites_this
//    - avg_weighted = 100 * total_mites / total_bees
//    - Update LED status based on avg_weighted threshold
//    - Finalize a "round" when g_round_bees reaches TARGET_BEES_PER_ROUND
//
// UI responsiveness:
//    - web_pump() is called between steps; should_abort() exits early if the
//      UI disables inference mid-cycle.
// =============================================================

static void run_inference_once() {
  web_pump();
  if (should_abort()) return;

  sdlog_printf("\n=== CYCLE frame=%lu millis=%lu ===\n", (unsigned long)g_frame_counter, (unsigned long)millis());

  if (!ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf)) {
    sdlog_printf("CYCLE fail capture\n");
    g_frame_counter++;
    return;
  }

  web_pump();
  if (should_abort()) return;

  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_get_data;

  ei_impulse_result_t result = { 0 };
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);

  web_pump();
  if (should_abort()) return;

  if (err != EI_IMPULSE_OK) {
    sdlog_printf("CYCLE fail run_classifier err=%d\n", err);
    g_frame_counter++;
    return;
  }

  log_bee_detections(result);
  save_bee_overlay_to_sd(result);

  const uint32_t bees_this = count_bee_detections(result);

  const bool got_centers = write_centers_txt(result);
  web_pump();
  if (should_abort()) return;

  if (got_centers) save_crops();

  uint32_t mites_this = 0;
  web_pump();
  if (should_abort()) return;

  if (got_centers && g_crop_count > 0) mites_this = run_varroa_on_new_crops_and_count();
  else sdlog_printf("VARROA skip got_centers=%d crops=%lu\n", (int)got_centers, (unsigned long)g_crop_count);

  g_round_bees  += bees_this;
  g_round_mites += mites_this;
  g_total_bees  += bees_this;
  g_total_mites += mites_this;

  led_update_from_avg_weighted();

  sdlog_printf("CYCLE_SUMMARY bees=%lu mites=%lu | round bees=%lu/%lu mites=%lu | totals bees=%lu mites=%lu\n",
               (unsigned long)bees_this,
               (unsigned long)mites_this,
               (unsigned long)g_round_bees,
               (unsigned long)TARGET_BEES_PER_ROUND,
               (unsigned long)g_round_mites,
               (unsigned long)g_total_bees,
               (unsigned long)g_total_mites);

  maybe_finalize_round_and_log();
  g_frame_counter++;
}

// ================================
// Arduino setup / loop
// ================================
void setup() {
  Serial.begin(115200);
  Serial.println("EI Inferencing - SD overlay + counting + rotating logs + web UI");

  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS);
  strip.show(); // all off initially
  led_update_from_avg_weighted(true);

  if (!ei_camera_init()) {
    Serial.println("Failed to initialize Camera!");
  }

  // --- allocate buffers ---
  size_t in_bytes = (size_t)EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * EI_CAMERA_FRAME_BYTE_SIZE;
  snapshot_buf = (uint8_t*)ps_malloc(in_bytes);
  if (!snapshot_buf) snapshot_buf = (uint8_t*)malloc(in_bytes);
  if (!snapshot_buf) { Serial.println("ERR: snapshot_buf alloc!"); while (true) delay(1000); }

  size_t full_bytes = (size_t)FULL_W * FULL_H * 3;
  g_fullstage_buf = (uint8_t*)ps_malloc(full_bytes);
  if (!g_fullstage_buf) g_fullstage_buf = (uint8_t*)malloc(full_bytes);
  if (!g_fullstage_buf) { Serial.println("ERR: full buffer alloc!"); while (true) delay(1000); }

  g_crop_rgb = (uint8_t*)ps_malloc((size_t)CROP_SIZE * CROP_SIZE * 3);
  if (!g_crop_rgb) g_crop_rgb = (uint8_t*)malloc((size_t)CROP_SIZE * CROP_SIZE * 3);
  if (!g_crop_rgb) { Serial.println("ERR: crop buffer alloc!"); while (true) delay(1000); }

  size_t ov_bytes = (size_t)EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 3;
  g_bee_overlay_buf = (uint8_t*)ps_malloc(ov_bytes);
  if (!g_bee_overlay_buf) g_bee_overlay_buf = (uint8_t*)malloc(ov_bytes);
  if (!g_bee_overlay_buf) { Serial.println("ERR: bee overlay buffer alloc!"); while (true) delay(1000); }

  // --- SD + web UI ---
  g_sd_ok = init_sdcard();
  if (!g_sd_ok) {
    Serial.println("SD NOT AVAILABLE.");
  } else {
    if (!init_boot_session_dirs_and_log()) {
      Serial.println("BOOT SESSION INIT FAILED (dirs/log).");
      g_save_enabled = false;
      g_infer_enabled = false;
    } else {
      sdlog_printf("LOG_PATH=%s\n", g_log_path);
    }
    sd_web_ui_begin();
    g_web_started = true;
  }

  // --- varroa buffers ---
  size_t var_bytes = (size_t)EI_VARROA_INPUT_WIDTH * EI_VARROA_INPUT_HEIGHT * 3;
  g_var_snapshot_buf = (uint8_t*)ps_malloc(var_bytes);
  if (!g_var_snapshot_buf) g_var_snapshot_buf = (uint8_t*)malloc(var_bytes);

  g_var_overlay_buf = (uint8_t*)ps_malloc(var_bytes);
  if (!g_var_overlay_buf) g_var_overlay_buf = (uint8_t*)malloc(var_bytes);

  if (!g_var_snapshot_buf || !g_var_overlay_buf) {
    sdlog_printf("ERR: varroa buffers alloc!\n");
    while (true) delay(1000);
  }

  last_infer_ms = 0;
}

void loop() {
  web_pump();

  if (!g_infer_enabled) {
    delay(5);
    return;
  }

  const uint32_t now = millis();
  if (last_infer_ms == 0 || (now - last_infer_ms) >= INFER_PERIOD_MS) {
    last_infer_ms = now;
    run_inference_once();
  }
}
