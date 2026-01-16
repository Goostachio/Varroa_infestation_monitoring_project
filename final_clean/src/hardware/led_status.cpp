#include "led_status.h"

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

void led_init() {
  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS);
  strip.show();
  led_update_from_avg_weighted(true);
}

void led_update_from_avg_weighted(bool force) {
  const double avg_w = avg_weighted_pct_boot();
  const LedState target = (avg_w > LED_INFEST_THRESH_PCT) ? LedState::Red : LedState::Green;

  if (!force && target == g_led_state) return;
  g_led_state = target;

  if (target == LedState::Red) led_set_rgb(255, 0, 0);
  else                        led_set_rgb(0, 255, 0);
}
