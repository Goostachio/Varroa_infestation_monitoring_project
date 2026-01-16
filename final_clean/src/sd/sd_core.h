#pragma once
#include "../globals.h"

bool sd_init_mount();
bool sd_init_boot_session_dirs_and_log();

void sdlog_printf(const char* fmt, ...);

bool sd_wipe_dir_contents(const char* dir_path);
bool sd_copy_file(const char* src_path, const char* dst_path);

bool sd_write_jpg_rgb888(const char* out_path, const uint8_t* rgb, int W, int H, int quality);
bool sd_save_fb_jpeg(camera_fb_t* fb);
