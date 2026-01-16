#include <merge_b.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"
#include "img_converters.h"
#include "src/ui/sd_web_ui.h"

#include "src/globals.h"
#include "src/sd/sd_core.h"
#include "src/ui/ui_web.h"
#include "src/camera/camera_ei.h"
#include "src/hardware/led_status.h"
#include "pipeline.h"

void setup() {
  Serial.begin(115200);
  Serial.println("EI Inferencing - modular skeleton (SD + UI + pipeline)");

  led_init();

  // allocate buffers
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

  size_t var_bytes = (size_t)EI_VARROA_INPUT_WIDTH * EI_VARROA_INPUT_HEIGHT * 3;
  g_var_snapshot_buf = (uint8_t*)ps_malloc(var_bytes);
  if (!g_var_snapshot_buf) g_var_snapshot_buf = (uint8_t*)malloc(var_bytes);

  g_var_overlay_buf = (uint8_t*)ps_malloc(var_bytes);
  if (!g_var_overlay_buf) g_var_overlay_buf = (uint8_t*)malloc(var_bytes);

  if (!g_var_snapshot_buf || !g_var_overlay_buf) {
    Serial.println("ERR: varroa buffers alloc!");
    while (true) delay(1000);
  }

  // camera
  if (!camera_init_ei()) Serial.println("Failed to initialize Camera!");

  // SD + session + UI
  g_sd_ok = sd_init_mount();
  if (!g_sd_ok) {
    Serial.println("SD NOT AVAILABLE.");
  } else {
    if (!sd_init_boot_session_dirs_and_log()) {
      Serial.println("BOOT SESSION INIT FAILED (dirs/log).");
      g_save_enabled = false;
      g_infer_enabled = false;
    } else {
      sdlog_printf("LOG_PATH=%s\n", g_log_path);
    }
    web_begin();
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
    pipeline_run_once();
  }
}
