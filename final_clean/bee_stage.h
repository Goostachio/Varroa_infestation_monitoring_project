#pragma once
#include "src/globals.h"
#include <merge_b.h>

uint32_t bee_count_detections(const ei_impulse_result_t& res);
void bee_log_detections(const ei_impulse_result_t& res);
void bee_save_overlay(const ei_impulse_result_t& res);
bool bee_write_centers_txt(const ei_impulse_result_t& res);
