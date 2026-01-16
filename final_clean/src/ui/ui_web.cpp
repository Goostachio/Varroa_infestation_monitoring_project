#include "ui_web.h"
#include "sd_web_ui.h"

void web_begin() {
  sd_web_ui_begin();
  g_web_started = true;
}

void web_pump() {
  if (g_web_started) sd_web_ui_loop();
}
