#pragma once
#include "../globals.h"

bool camera_init_ei();
bool camera_capture_ei(uint32_t img_width, uint32_t img_height, uint8_t* out_buf);
