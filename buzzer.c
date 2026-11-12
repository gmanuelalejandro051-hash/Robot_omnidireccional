#include "buzzer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "BUZZER";

/* Store config for later use (e.g. changing frequency) */
static buzzer_config_t s_buzzer_cfg;
static bool s_initialized = false;

/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */

esp_err_t buzzer_init(const buzzer_config_t *config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "buzzer_init: config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    s_buzzer_cfg = *config;

    /* Configure LEDC timer */
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num       = s_buzzer_cfg.timer_num,
        .freq_hz         = s_buzzer_cfg.freq_hz,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Configure LEDC channel — start with duty = 0 (silent) */
    ledc_channel_config_t channel_cfg = {
        .gpio_num   = s_buzzer_cfg.gpio_num,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = s_buzzer_cfg.channel,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = s_buzzer_cfg.timer_num,
        .duty       = 0,
    };
    ret = ledc_channel_config(&channel_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Buzzer initialized on GPIO %d, freq %lu Hz",
             s_buzzer_cfg.gpio_num, (unsigned long)s_buzzer_cfg.freq_hz);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */

esp_err_t buzzer_play_tone(uint32_t freq_hz)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Buzzer not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Update frequency */
    esp_err_t ret = ledc_set_freq(LEDC_LOW_SPEED_MODE, s_buzzer_cfg.timer_num, freq_hz);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_set_freq failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 50 % duty cycle for a square wave — max duty at 10-bit = 1023 */
    ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, s_buzzer_cfg.channel, 512);
    if (ret != ESP_OK) return ret;

    ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, s_buzzer_cfg.channel);
    if (ret != ESP_OK) return ret;

    ESP_LOGD(TAG, "Playing tone at %lu Hz", (unsigned long)freq_hz);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */

esp_err_t buzzer_stop(void)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Buzzer not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, s_buzzer_cfg.channel, 0);
    if (ret != ESP_OK) return ret;

    ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, s_buzzer_cfg.channel);
    if (ret != ESP_OK) return ret;

    ESP_LOGD(TAG, "Buzzer stopped");
    return ESP_OK;
}

/* ------------------------------------------------------------------ */

esp_err_t buzzer_beep(uint32_t freq_hz, uint32_t duration_ms)
{
    esp_err_t ret = buzzer_play_tone(freq_hz);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(duration_ms));

    return buzzer_stop();
}

/* ------------------------------------------------------------------ */

esp_err_t buzzer_beep_pattern(uint32_t count, uint32_t freq_hz,
                               uint32_t on_ms, uint32_t off_ms)
{
    for (uint32_t i = 0; i < count; i++) {
        esp_err_t ret = buzzer_beep(freq_hz, on_ms);
        if (ret != ESP_OK) return ret;

        if (i < count - 1) {
            vTaskDelay(pdMS_TO_TICKS(off_ms));
        }
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */

esp_err_t buzzer_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }

    buzzer_stop();
    ledc_stop(LEDC_LOW_SPEED_MODE, s_buzzer_cfg.channel, 0);
    s_initialized = false;
    ESP_LOGI(TAG, "Buzzer deinitialized");
    return ESP_OK;
}