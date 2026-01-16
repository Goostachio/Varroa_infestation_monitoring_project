#pragma once
#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>
#include <esp_camera.h>
#include <Adafruit_NeoPixel.h>

#include "app_config.h"

// -------------------------------
// Globals that UI may depend on
// -------------------------------
extern volatile bool g_infer_enabled;
extern volatile bool g_save_enabled;
extern bool g_web_started;

// -------------------------------
// Runtime state
// -------------------------------
extern bool is_initialised;
extern bool debug_nn;

extern uint8_t* snapshot_buf;
extern uint8_t* g_fullstage_buf;
extern uint8_t* g_crop_rgb;
extern uint8_t* g_bee_overlay_buf;

extern uint16_t g_full_w;
extern uint16_t g_full_h;

extern char g_last_frame_path[128];
extern char g_last_meta_path[128];
extern uint32_t g_frame_counter;

extern bool g_sd_ok;

// timing
extern uint32_t INFER_PERIOD_MS;
extern uint32_t last_infer_ms;

// -------------------------------
// Boot session dirs + log
// -------------------------------
extern char g_frames_dir[64];
extern char g_bee_overlays_dir[64];
extern char g_crops_dir[64];
extern char g_overlays_dir[64];
extern char g_overlays_mite_dir[96];
extern char g_overlays_nomite_dir[96];

extern const char* LOG_DIR;
extern const char* BOOT_ID_PATH;
extern const char* OVERLAY_MITE_SUBDIR;
extern const char* OVERLAY_NO_MITE_SUBDIR;

extern uint32_t g_boot_id;
extern char g_log_path[128];
extern File g_log_file;

extern const bool LOG_ECHO_SERIAL;

// -------------------------------
// Crop bookkeeping
// -------------------------------
struct CropMeta { uint32_t bbox_index; char path[128]; };
extern CropMeta g_crop_meta[MAX_CROPS];
extern uint32_t g_crop_count;

// -------------------------------
// Varroa buffers
// -------------------------------
extern uint8_t* g_var_snapshot_buf;
extern uint8_t* g_var_overlay_buf;

// -------------------------------
// Counting
// -------------------------------
extern uint32_t g_round_bees;
extern uint32_t g_round_mites;
extern volatile uint32_t g_total_bees;
extern volatile uint32_t g_total_mites;

// -------------------------------
// LED object (optional global)
// -------------------------------
extern Adafruit_NeoPixel strip;

// -------------------------------
// Small helpers
// -------------------------------
inline bool sd_writes_enabled() { return g_sd_ok && g_save_enabled; }
inline bool should_abort() { return !g_infer_enabled; }
