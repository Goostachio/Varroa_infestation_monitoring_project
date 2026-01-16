#pragma once
#include <stdint.h>

// ================================
// Hardware / Pin Configuration
// ================================
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5
#define Y2_GPIO_NUM    11
#define Y3_GPIO_NUM    9
#define Y4_GPIO_NUM    8
#define Y5_GPIO_NUM    10
#define Y6_GPIO_NUM    12
#define Y7_GPIO_NUM    18
#define Y8_GPIO_NUM    17
#define Y9_GPIO_NUM    16
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

#define SD_CLK_PIN     39
#define SD_CMD_PIN     38
#define SD_DATA0_PIN   40

#define LED_PIN        21
#define LED_COUNT      1

// ================================
// Image / Model Dimensions
// ================================
#define FULL_W 1280
#define FULL_H 1024
#define EI_CAMERA_FRAME_BYTE_SIZE 3

static constexpr int CROP_SIZE = 160;
static constexpr int MAX_CROPS = 50;

// ================================
// Thresholds / Timing
// ================================
static constexpr float UI_THRESH      = 0.35f;
static constexpr float VAR_THRESH     = 0.50f;
static constexpr int   DROPPED_FRAMES = 10;

static constexpr int JPEG_QUALITY = 100;

// ================================
// Counting
// ================================
static constexpr uint32_t TARGET_BEES_PER_ROUND = 100;

// ================================
// LED logic
// ================================
static constexpr double  LED_INFEST_THRESH_PCT = 10.0;
static constexpr uint8_t LED_BRIGHTNESS = 50;
