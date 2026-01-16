#include "sd_core.h"
#include "../util.h"
#include "img_converters.h"   // fmt2jpg, fmt2rgb888

#include <stdarg.h>
#include <memory>

// small internal helpers
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
  if (!sd_wipe_dir_contents(dir_path)) return false;
  return SD_MMC.rmdir(dir_path);
}

// copy buffer
static constexpr size_t SD_COPY_BUF_SIZE = 1024;
static uint8_t g_sd_copy_buf[SD_COPY_BUF_SIZE];

// boot id helpers
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

void sdlog_printf(const char* fmt, ...) {
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

bool sd_init_mount() {
  SD_MMC.setPins(SD_CLK_PIN, SD_CMD_PIN, SD_DATA0_PIN);
  if (!SD_MMC.begin("/sdcard", true, false)) {
    Serial.println("SD_MMC.begin() failed.");
    return false;
  }
  Serial.println("SD card mounted: /sdcard");
  return true;
}

bool sd_wipe_dir_contents(const char* dir_path) {
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

bool sd_copy_file(const char* src_path, const char* dst_path) {
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

bool sd_write_jpg_rgb888(const char* out_path, const uint8_t* rgb, int W, int H, int quality) {
  if (!sd_writes_enabled() || !out_path || !rgb || W <= 0 || H <= 0) return false;

  uint8_t* jbuf = nullptr;
  size_t jlen = 0;
  const bool enc = fmt2jpg(
    (uint8_t*)rgb,
    (size_t)W * (size_t)H * 3u,
    (uint16_t)W,
    (uint16_t)H,
    PIXFORMAT_RGB888,
    (uint8_t)quality,
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

bool sd_save_fb_jpeg(camera_fb_t* fb) {
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

bool sd_init_boot_session_dirs_and_log() {
  if (!g_sd_ok) return false;

  if (!ensure_dir("/frames")) return false;
  if (!ensure_dir("/bee_overlays")) return false;
  if (!ensure_dir("/crops")) return false;
  if (!ensure_dir("/overlays")) return false;
  if (!ensure_dir(LOG_DIR)) return false;

  if (!sd_wipe_dir_contents("/frames")) return false;
  if (!sd_wipe_dir_contents("/crops")) return false;

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
