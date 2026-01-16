#include "camera_ei.h"
#include "../sd/sd_core.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "img_converters.h"

// camera config
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

bool camera_init_ei() {
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

bool camera_capture_ei(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
  if (!is_initialised) { sdlog_printf("ERR camera not initialized\n"); return false; }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { sdlog_printf("ERR camera capture failed\n"); return false; }

  g_full_w = fb->width;
  g_full_h = fb->height;

  (void)sd_save_fb_jpeg(fb);

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
