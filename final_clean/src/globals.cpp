#include "globals.h"

// UI flags
volatile bool g_infer_enabled = false;
volatile bool g_save_enabled  = false;
bool g_web_started = false;

// runtime state
bool is_initialised = false;
bool debug_nn = false;

uint8_t* snapshot_buf      = nullptr;
uint8_t* g_fullstage_buf   = nullptr;
uint8_t* g_crop_rgb        = nullptr;
uint8_t* g_bee_overlay_buf = nullptr;

uint16_t g_full_w = FULL_W;
uint16_t g_full_h = FULL_H;

char g_last_frame_path[128] = {0};
char g_last_meta_path[128]  = {0};
uint32_t g_frame_counter = 0;

bool g_sd_ok = false;

// timing
uint32_t INFER_PERIOD_MS = 5000;
uint32_t last_infer_ms = 0;

// boot session dirs + log
char g_frames_dir[64]       = {0};
char g_bee_overlays_dir[64] = {0};
char g_crops_dir[64]        = {0};
char g_overlays_dir[64]     = {0};
char g_overlays_mite_dir[96]   = {0};
char g_overlays_nomite_dir[96] = {0};

const bool LOG_ECHO_SERIAL = true;

const char* LOG_DIR = "/logs";
const char* BOOT_ID_PATH = "/logs/boot_id.txt";
const char* OVERLAY_MITE_SUBDIR = "mite";
const char* OVERLAY_NO_MITE_SUBDIR = "no_mite";

uint32_t g_boot_id = 0;
char g_log_path[128] = {0};
File g_log_file;

// crop meta
CropMeta g_crop_meta[MAX_CROPS];
uint32_t g_crop_count = 0;

// varroa buffers
uint8_t* g_var_snapshot_buf = nullptr;
uint8_t* g_var_overlay_buf  = nullptr;

// counting
uint32_t g_round_bees  = 0;
uint32_t g_round_mites = 0;
volatile uint32_t g_total_bees  = 0;
volatile uint32_t g_total_mites = 0;

// LED
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
