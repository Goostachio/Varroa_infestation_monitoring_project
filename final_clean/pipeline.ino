#include "pipeline.h"
#include "src/ui/ui_web.h"
#include "src/camera/camera_ei.h"
#include "bee_stage.h"
#include "crop_stage.h"
#include "varroa_stage.h"
#include "src/hardware/led_status.h"
#include "src/sd/sd_core.h"

#include <merge_b.h>
#include "src/ei/ei_signal_shim.h"


static void maybe_finalize_round_and_log() {
  if (g_round_bees < TARGET_BEES_PER_ROUND) return;

  const double round_pct = (g_round_bees > 0)
    ? (100.0 * (double)g_round_mites / (double)g_round_bees)
    : 0.0;

  const double avg_weighted = (g_total_bees > 0)
    ? (100.0 * (double)g_total_mites / (double)g_total_bees)
    : 0.0;

  sdlog_printf("[ROUND DONE] bees=%lu mites=%lu => %.2f%% | avg_weighted=%.2f%% | totals bees=%lu mites=%lu\n",
               (unsigned long)g_round_bees,
               (unsigned long)g_round_mites,
               round_pct,
               avg_weighted,
               (unsigned long)g_total_bees,
               (unsigned long)g_total_mites);

  g_round_bees = 0;
  g_round_mites = 0;
}

void pipeline_run_once() {
  web_pump();
  if (should_abort()) return;

  sdlog_printf("\n=== CYCLE frame=%lu millis=%lu ===\n",
               (unsigned long)g_frame_counter, (unsigned long)millis());

  if (!camera_capture_ei(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf)) {
    sdlog_printf("CYCLE fail capture\n");
    g_frame_counter++;
    return;
  }

  web_pump();
  if (should_abort()) return;

  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_bee_get_data;

  ei_impulse_result_t result = { 0 };
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);

  web_pump();
  if (should_abort()) return;

  if (err != EI_IMPULSE_OK) {
    sdlog_printf("CYCLE fail run_classifier err=%d\n", err);
    g_frame_counter++;
    return;
  }

  bee_log_detections(result);
  bee_save_overlay(result);

  const uint32_t bees_this = bee_count_detections(result);

  const bool got_centers = bee_write_centers_txt(result);
  web_pump();
  if (should_abort()) return;

  if (got_centers) crops_save_from_last_frame();

  uint32_t mites_this = 0;
  web_pump();
  if (should_abort()) return;

  if (got_centers && g_crop_count > 0) mites_this = varroa_run_on_new_crops_and_count();
  else sdlog_printf("VARROA skip got_centers=%d crops=%lu\n", (int)got_centers, (unsigned long)g_crop_count);

  g_round_bees  += bees_this;
  g_round_mites += mites_this;
  g_total_bees  += bees_this;
  g_total_mites += mites_this;

  led_update_from_avg_weighted();

  sdlog_printf("CYCLE_SUMMARY bees=%lu mites=%lu | round bees=%lu/%lu mites=%lu | totals bees=%lu mites=%lu\n",
               (unsigned long)bees_this,
               (unsigned long)mites_this,
               (unsigned long)g_round_bees,
               (unsigned long)TARGET_BEES_PER_ROUND,
               (unsigned long)g_round_mites,
               (unsigned long)g_total_bees,
               (unsigned long)g_total_mites);

  maybe_finalize_round_and_log();
  g_frame_counter++;
}