# Varroa Infestation Monitoring (ESP32-S3 + OV2640)

This repository contains an Arduino sketch (`final_clean/final_clean.ino`) that runs an Edge Impulse image pipeline on an **ESP32-S3-N16R8** camera module with an **OV2640** sensor, logs results to microSD, and serves a lightweight Wi-Fi web UI from the device.

## Description

This project implements a two-stage on-device TinyML vision pipeline for monitoring *Varroa destructor* mites on honey bees using an ESP32-S3-CAM + OV2640 camera. The system runs fully offline on the microcontroller: it captures images, performs inference locally, logs results to microSD, and provides user feedback via a Web UI + NeoPixel LED.

## Table of contents

* [Features](#features)
* [Motivation / Background](#motivation--background)
* [FOMO Model Training (Edge Impulse)](#fomo-model-training-edge-impulse)
* [System Overview (Two-Stage Inference)](#system-overview-two-stage-inference)
* [Hardware & Wiring](#hardware--wiring)
* [Usage](#usage)
* [Results](#results)
* [Limitations / Known Issues](#limitations--known-issues)
* [Contact / Support](#contact--support)

## Features

* Two-stage inference:
  * Stage 1: Bee detection on the full frame (FOMO model).
  * Stage 2: Varroa detection on per-bee crops (second impulse).
* Multi-impulse Arduino deployment (Single SDK + Dual Impulses) to avoid linker conflicts, using explicit impulse handles:
  * `ei_bee_impulse()` and `ei_varroa_impulse()`.
* Runtime inference routing via `process_impulse()` with the chosen impulse handle (instead of relying on default `run_classifier()` macros).
* Dynamic input handling (different model input sizes) by avoiding ambiguous global macros and resizing/cropping based on the active impulse metadata.
* SD logging + visual audit trail:
  * Saves raw frames as timestamped JPEGs.
  * Saves overlay/annotated frames and crops.
  * Appends log entries with counts/confidences.
* Infestation metric + alerting:
  * Computes cumulative infestation percentage as a weighted metric (mites / bees).
  * NeoPixel LED feedback (green normal, red high infestation).
* Web UI control over Wi-Fi for monitoring and toggling inference/saving.

## Motivation / Background

The project targets a low-cost, accessible monitoring approach for small-scale beekeepers, prioritizing offline inference (privacy, low latency, remote operation). Varroa mites are a major threat to colonies, and continuous monitoring is difficult with manual methods; this work explores embedded detection directly on microcontroller hardware.

## FOMO Model Training (Edge Impulse)

Backbone: MobileNetV2 (chosen for efficiency on microcontrollers).

Bee detector (FOMO):

* Width multiplier: 0.1
* Epochs: 30
* Learning rate: 0.01
* Cut-point: block-6-expand-relu
* Input resolution: 320×320
* Squash: enabled (reduce size / memory footprint)

Varroa detector (FOMO):

* Motivation: mites are very small and low contrast, requiring a more expressive backbone.
* Width multiplier: 0.35
* Epochs: 60
* Learning rate: 0.01
* Cut-point: block-6-expand-relu
* Input resolution: 160×160
* FOMO filters: 16
* Two-stage filtering enabled for stability/accuracy

## System Overview (Two-Stage Inference)

Stage 1 — Bee detection (full frame):

* Capture full-resolution frame as JPEG (SXGA 1280×1024), decode to RGB, resize to the bee model input, then run the bee impulse.
* Detections filtered by `BEE_THRESH = 0.50`.

Stage 2 — Varroa detection (per-bee crops):

* Map bee-center coordinates back into full-resolution space, extract fixed-size crops (`CROP_SIZE = 160`), resize to varroa input (160×160), then run the varroa impulse.
* Detections filtered by `VAR_THRESH = 0.50`.

Scheduler & controls:

* A periodic scheduler runs one full cycle every `INFER_PERIOD_MS` (default 5000 ms).
* Web UI can pause/stop inference (`g_infer_enabled`) and toggle SD writes (`g_save_enabled`).

## Hardware & Wiring

### Hardware requirements

* **ESP32-S3-N16R8** camera module (PSRAM required).
* **OV2640** camera module.
* **microSD card** (formatted FAT32).
* **NeoPixel-compatible status LED** (1x RGB LED).
* Power supply and wiring per the pin map below.

### Core components

* ESP32-S3-WROOM (N16R8): 16MB flash + 8MB PSRAM required for the 2-stage pipeline and buffering snapshots.
* OV2640 (2MP) camera with manual-focus lens (to focus at ~10–15 cm).
* microSD 8GB required (stores full-resolution frames for cropping + logs).
* WS2812B (NeoPixel) RGB LED for health/alert status.

### Wiring (NeoPixel)

* 5V → WS2812B VCC
* GND → WS2812B GND
* GPIO 21 → WS2812B DIN via 330Ω series resistor

### Storage / IO configuration

* microSD is interfaced via SDMMC (1-bit mode).
## Pin map (from firmware)

The firmware pins are defined in `final_clean/src/app_config.h`. Update these if your wiring differs.

### Camera (OV2640)

| Signal | GPIO |
| --- | --- |
| XCLK | 15 |
| SIOD (SDA) | 4 |
| SIOC (SCL) | 5 |
| Y2 | 11 |
| Y3 | 9 |
| Y4 | 8 |
| Y5 | 10 |
| Y6 | 12 |
| Y7 | 18 |
| Y8 | 17 |
| Y9 | 16 |
| VSYNC | 6 |
| HREF | 7 |
| PCLK | 13 |
| PWDN | -1 (not used) |
| RESET | -1 (not used) |

### microSD (SD_MMC, 1-bit mode)

| Signal | GPIO |
| --- | --- |
| SD_CLK | 39 |
| SD_CMD | 38 |
| SD_DATA0 | 40 |

### NeoPixel LED

| Signal | GPIO |
| --- | --- |
| LED Data | 21 |

### Prerequisites

* Arduino IDE workflow on ESP32-S3.
* A merged Edge Impulse Arduino library using the Single SDK + Dual Impulse structure (see next section).

### 1) Install the ESP32 board package

Install **Espressif ESP32** support in Arduino IDE (Boards Manager). Use the board manager URL below:

* `https://espressif.github.io/arduino-esp32/package_esp32_index.json`

The sketch relies on:

* `WiFi.h`, `WebServer.h`
* `esp_camera.h`
* `FS.h`, `SD_MMC.h`
* `img_converters.h` (from the ESP32 camera support)

These are part of the Arduino-ESP32 core.

### Library integration (Single SDK + Dual Impulses)

* Keep one `edge-impulse-sdk/` directory shared by both models.
* Merge both projects’ `model-parameters/` and `tflite-model/` into one library (preserve unique model blob headers).
* Combine both projects’ `model_variables.h` contents (unique project IDs allow coexistence).
* Expose two explicit impulse handles: `ei_bee_impulse()` and `ei_varroa_impulse()`.

### 2) Install required libraries

* **Adafruit NeoPixel** (`Adafruit_NeoPixel.h`).
* **Edge Impulse exported library** included in this repo: `libraries/merge_b.zip`.

To install the Edge Impulse library in Arduino IDE:

1. **Sketch → Include Library → Add .ZIP Library...**
2. Select `libraries/merge_b.zip` from this repository.

### 3) Board and build settings

In **Tools** menu (Arduino IDE):

* **Board:** ESP32S3 Dev Module (or your ESP32-S3-N16R8 variant)
* **PSRAM:** OPI PSRAM (required)
* **Flash Size:** 16MB
* **Partition Scheme:** Huge APP (required)

### 4) Open and upload

1. Open `final_clean/final_clean.ino`.
2. Select the correct serial port.
3. Upload.

## Usage

* On boot, the device mounts the SD card and starts a Wi-Fi AP.
* **Wi-Fi SSID:** `ESP32-SD`
* **Wi-Fi password:** (open / none)
* Open the serial monitor to see the assigned IP and log output.

The web UI is served from the device and shows:

* Current inference state (enabled/disabled)
* Rolling counts of bees vs. mites
* SD-backed image browsing APIs

### Using the web UI

1. Connect your phone or laptop to the `ESP32-SD` Wi-Fi network.
2. Open a browser and visit default assigned IP.
3. Click **Start** to begin inferencing.
4. Click **Stop** to stop inferencing and browse the latest saved images of detected bees and varroa mites.

### What happens each inference cycle

* Capture frame (JPEG SXGA), decode to RGB.
* Resize/crop to bee input and run Stage 1.
* For each detected bee, crop 160×160 from full-res and run Stage 2.
* Log images + results to SD; update infestation metric; update LED; serve updated stats in Web UI.

### Outputs saved to SD

* Timestamped raw JPEG frames (audit trail).
* Overlay/annotated frames (bee boxes, mite indicators).
* Crops and mite overlays (for review).

### Alerts

* LED turns red when infestation exceeds 10%.

## Web UI screenshots

![Web UI showing bee detection](images/bee.png)
![Web UI showing varroa mite detection](images/mite.png)
![Web UI showing no-mite detection](images/no-mite.png)

## Wiring diagram

![Hardware wiring diagram](images/wiring.png)

## Project structure (high level)

* `final_clean/`: Arduino sketch folder (open `final_clean/final_clean.ino` to run inference).
* `final_clean/src/`: camera, SD card, UI, and pipeline logic.
* `libraries/merge_b.zip`: Edge Impulse library export.
* `merger/`: helper Python code used to merge and produce `merge_b.zip`.

## Results

Offline (test set) training metrics:

* Bee detector: Precision 0.96, Recall 0.94, F1 0.95
* Varroa detector: Precision 0.85, Recall 0.94, F1 0.89

On-device test (ESP32-S3):

* Varroa detection (40 samples visual count): TP=47, FP=14, FN=46 → Precision 0.77, Recall 0.51, F1 0.61

## Limitations / Known Issues

* Dataset limitation (real infestation scarcity): field collection missed infestation periods; mites not always present and seasonal.
* Synthetic / public dataset issues: reliance on edited “infested” samples and public datasets, with limited realism and annotation quality issues (miss-labels, cut-off/tightness problems).
* Training limitation (Edge Impulse free tier): developer plan training jobs limited to 60 minutes, forcing smaller training sets (~1000–1500 images).
* Evaluation limitation: many tests were done using photos of test images on a computer screen or by directly passing images, and real-world testing was constrained.
* Performance gap (training vs on-device): Varroa model drops from F1 0.89 (offline) to 0.61 on-device.

## Contact / Support

Create an issue in this repository for questions and bug reports, or contact: 10422051@student.vgu.edu.vn

## Notes

* The camera configuration uses `FRAMESIZE_SXGA` (1280x1024) and saves JPEG frames to the SD card.
* NeoPixel status LED: green when infestation rate ≤ threshold, red above threshold.

## Dataset and Edge Impulse models

* Dataset (Google Drive): https://drive.google.com/drive/folders/1YUVjjQaT-wlv3pP-dXyxlRmlFddlCYIB?usp=drive_link
* Edge Impulse models:
  * https://studio.edgeimpulse.com/public/872791/live
  * https://studio.edgeimpulse.com/public/874563/live
