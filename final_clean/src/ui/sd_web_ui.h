#pragma once
#include <stdint.h>

extern volatile bool g_infer_enabled;
extern volatile bool g_save_enabled;

extern volatile uint32_t g_total_bees;
extern volatile uint32_t g_total_mites;

void sd_web_ui_begin();
void sd_web_ui_loop();