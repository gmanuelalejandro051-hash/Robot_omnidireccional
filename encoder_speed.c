
#include "encoder_speed.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "driver/pulse_cnt.h"   // ESP-IDF v5.x PCNT driver
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
static const char *TAG = "encoder";

// ── Private state ─────────────────────────────────────────────────────────────
struct encoder_t {
    encoder_config_t    cfg;

    // PCNT objects (ESP-IDF v5 API)
    pcnt_unit_handle_t  pcnt_unit;
    pcnt_channel_handle_t chan_a;
    pcnt_channel_handle_t chan_b;

    // Accumulated count (PCNT is 16-bit; we extend it via overflow watchpoint)
    volatile int64_t    accum_count;    // total pulses, extended
    int                 overflow_limit; // watchpoint value (half of int16 range)

    // Speed calculation
    int64_t             prev_count;     // count at last speed sample
    int64_t             prev_time_us;   // timestamp at last speed sample
    float               speed_rpm;      // last computed speed
    float               speed_rad_s;
};

// ── Overflow watchpoint callback ──────────────────────────────────────────────
// PCNT is a 16-bit counter. When it hits the watchpoint we fold it back
// and add the delta to accum_count so we never lose pulses.
static bool IRAM_ATTR pcnt_on_reach(pcnt_unit_handle_t unit,
                                     const pcnt_watch_event_data_t *edata,
                                     void *user_ctx)
{
    struct encoder_t *enc = (struct encoder_t *)user_ctx;

    // edata->watch_point_val is the threshold that triggered
    // edata->zero_cross_mode tells us direction
    enc->accum_count += edata->zero_cross_mode;
    pcnt_unit_clear_count(unit);   // reset hardware counter, keep accumulator

    return false;  // no high-priority task wakeup needed
}

// ── Public API ────────────────────────────────────────────────────────────────

esp_err_t encoder_init(const encoder_config_t *cfg, encoder_handle_t *handle)
{
    ESP_RETURN_ON_FALSE(cfg && handle, ESP_ERR_INVALID_ARG, TAG, "null arg");

    struct encoder_t *enc = calloc(1, sizeof(*enc));
    ESP_RETURN_ON_FALSE(enc, ESP_ERR_NO_MEM, TAG, "alloc failed");

    memcpy(&enc->cfg, cfg, sizeof(*cfg));
    enc->overflow_limit = 20000;   // well within ±32767

    // ── 1. Create PCNT unit ───────────────────────────────────────────────
    pcnt_unit_config_t unit_cfg = {
        .low_limit  = -enc->overflow_limit,
        .high_limit =  enc->overflow_limit,
        .flags.accum_count = false,   // we do our own 64-bit accumulation
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &enc->pcnt_unit));

    // ── 2. Glitch filter (debounce) ───────────────────────────────────────
    // Ignores pulses shorter than ~1 µs — removes electrical noise
    pcnt_glitch_filter_config_t filter_cfg = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(enc->pcnt_unit, &filter_cfg));

    // ── 3. Channel A: counts on A edges, direction from B level ──────────
    pcnt_chan_config_t chan_a_cfg = {
        .edge_gpio_num  = cfg->gpio_a,
        .level_gpio_num = cfg->gpio_b,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(enc->pcnt_unit, &chan_a_cfg, &enc->chan_a));

    // Rising edge of A with B=LOW  → count up   (forward)
    // Rising edge of A with B=HIGH → count down  (reverse)
    // Falling edge of A with B=LOW  → count down
    // Falling edge of A with B=HIGH → count up
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(enc->chan_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,   // A falling + B=LOW  → -1
        PCNT_CHANNEL_EDGE_ACTION_INCREASE)); // A rising  + B=LOW  → +1

    ESP_ERROR_CHECK(pcnt_channel_set_level_action(enc->chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE,   // B=HIGH: flip polarity
        PCNT_CHANNEL_LEVEL_ACTION_KEEP));    // B=LOW:  keep as-is

    // ── 4. Channel B: counts on B edges, direction from A level ──────────
    // (Doubles the resolution — x4 quadrature decoding)
    pcnt_chan_config_t chan_b_cfg = {
        .edge_gpio_num  = cfg->gpio_b,
        .level_gpio_num = cfg->gpio_a,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(enc->pcnt_unit, &chan_b_cfg, &enc->chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(enc->chan_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,   // B falling + A=LOW  → +1
        PCNT_CHANNEL_EDGE_ACTION_DECREASE)); // B rising  + A=LOW  → -1

    ESP_ERROR_CHECK(pcnt_channel_set_level_action(enc->chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    // ── 5. Watchpoints for 64-bit overflow extension ──────────────────────
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(enc->pcnt_unit,
                     enc->overflow_limit));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(enc->pcnt_unit,
                    -enc->overflow_limit));

    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_on_reach,
    };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(enc->pcnt_unit,
                    &cbs, enc));

    // ── 6. Enable unit (does not start counting yet) ──────────────────────
    ESP_ERROR_CHECK(pcnt_unit_enable(enc->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(enc->pcnt_unit));

    enc->prev_time_us = esp_timer_get_time();

    *handle = enc;
    ESP_LOGI(TAG, "Encoder init OK  gpio_a=%d gpio_b=%d ppr=%d",
             cfg->gpio_a, cfg->gpio_b, cfg->pulses_per_rev);
    return ESP_OK;
}

esp_err_t encoder_start(encoder_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "null handle");
    return pcnt_unit_start(handle->pcnt_unit);
}

esp_err_t encoder_stop(encoder_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "null handle");
    return pcnt_unit_stop(handle->pcnt_unit);
}

esp_err_t encoder_get_count(encoder_handle_t handle, int64_t *count)
{
    ESP_RETURN_ON_FALSE(handle && count, ESP_ERR_INVALID_ARG, TAG, "null arg");

    int hw_count = 0;
    esp_err_t err = pcnt_unit_get_count(handle->pcnt_unit, &hw_count);
    if (err == ESP_OK) {
        *count = handle->accum_count + hw_count;
    }
    return err;
}

esp_err_t encoder_reset_count(encoder_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "null handle");
    handle->accum_count = 0;
    handle->prev_count  = 0;
    return pcnt_unit_clear_count(handle->pcnt_unit);
}

esp_err_t encoder_get_speed(encoder_handle_t handle,
                             float *rpm_out, float *rad_s_out)
{
    ESP_RETURN_ON_FALSE(handle && rpm_out, ESP_ERR_INVALID_ARG, TAG, "null arg");

    int64_t now_us = esp_timer_get_time();
    int64_t dt_us  = now_us - handle->prev_time_us;

    if (dt_us <= 0) {
        *rpm_out = handle->speed_rpm;
        if (rad_s_out) *rad_s_out = handle->speed_rad_s;
        return ESP_OK;
    }

    int64_t current_count = 0;
    encoder_get_count(handle, &current_count);

    int64_t delta_pulses = current_count - handle->prev_count;

    // x4 quadrature: effective PPR = cfg.pulses_per_rev * 4
    float effective_ppr = (float)(handle->cfg.pulses_per_rev) * 4.0f;

    // revolutions per microsecond → convert to RPM
    float rev_per_us  = (float)delta_pulses / (effective_ppr * (float)dt_us);
    float rpm         = rev_per_us * 60.0e6f;

    // rad/s = rpm * 2π / 60
    float rad_s = rpm * (2.0f * (float)M_PI / 60.0f);

    handle->speed_rpm   = rpm;
    handle->speed_rad_s = rad_s;
    handle->prev_count  = current_count;
    handle->prev_time_us = now_us;

    *rpm_out = rpm;
    if (rad_s_out) *rad_s_out = rad_s;
    return ESP_OK;
}

esp_err_t encoder_deinit(encoder_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "null handle");

    pcnt_unit_stop(handle->pcnt_unit);
    pcnt_unit_disable(handle->pcnt_unit);
    pcnt_del_channel(handle->chan_a);
    pcnt_del_channel(handle->chan_b);
    pcnt_del_unit(handle->pcnt_unit);
    free(handle);

    ESP_LOGI(TAG, "Encoder deinited");
    return ESP_OK;
}