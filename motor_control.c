#include "motor_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"

static const char *TAG = "l298_motor";

typedef struct {
    int                   pwm_gpio;
    int                   dir1_gpio;
    int                   dir2_gpio;
    mcpwm_timer_handle_t  timer;
    mcpwm_oper_handle_t   oper;
    mcpwm_cmpr_handle_t   cmpr;
    mcpwm_gen_handle_t    gen;
    uint32_t              period_ticks;
} l298_motor_priv_t;

struct l298_dev_s {
    l298_motor_priv_t motors[L298_MOTOR_MAX];
    uint32_t          pwm_freq_hz;
};

static inline uint32_t pct_to_ticks(uint32_t period_ticks, uint8_t pct)
{
    if (pct >= 100) return period_ticks;
    return (period_ticks * pct) / 100u;
}

static esp_err_t gpio_out_init(int gpio_num)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    return gpio_config(&cfg);
}

static void apply_direction(const l298_motor_priv_t *m, l298_direction_t dir)
{
    switch (dir) {
        case L298_FORWARD:
            gpio_set_level(m->dir1_gpio, 1);
            gpio_set_level(m->dir2_gpio, 0);
            break;
        case L298_BACKWARD:
            gpio_set_level(m->dir1_gpio, 0);
            gpio_set_level(m->dir2_gpio, 1);
            break;
        case L298_BRAKE:
            gpio_set_level(m->dir1_gpio, 1);
            gpio_set_level(m->dir2_gpio, 1);
            break;
        case L298_COAST:
        default:
            gpio_set_level(m->dir1_gpio, 0);
            gpio_set_level(m->dir2_gpio, 0);
            break;
    }
}

#define MCPWM_RESOLUTION_HZ  (1000000u)

static esp_err_t motor_mcpwm_init(l298_motor_priv_t *m, int motor_idx, uint32_t freq_hz)
{
    esp_err_t ret = ESP_OK;

    m->period_ticks = MCPWM_RESOLUTION_HZ / freq_hz;

    /* Motors A,B -> group 0 | Motor C -> group 1 */
    int group = (motor_idx < 2) ? 0 : 1;

    mcpwm_timer_config_t timer_cfg = {
        .group_id      = group,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MCPWM_RESOLUTION_HZ,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks  = m->period_ticks,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_timer(&timer_cfg, &m->timer),
                        TAG, "timer %d init failed", motor_idx);

    mcpwm_operator_config_t oper_cfg = { .group_id = group };
    ESP_RETURN_ON_ERROR(mcpwm_new_operator(&oper_cfg, &m->oper),
                        TAG, "operator %d init failed", motor_idx);

    ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(m->oper, m->timer),
                        TAG, "connect timer %d failed", motor_idx);

    mcpwm_comparator_config_t cmpr_cfg = { .flags.update_cmp_on_tez = true };
    ESP_RETURN_ON_ERROR(mcpwm_new_comparator(m->oper, &cmpr_cfg, &m->cmpr),
                        TAG, "comparator %d init failed", motor_idx);

    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(m->cmpr, 0),
                        TAG, "set compare 0 failed");

    mcpwm_generator_config_t gen_cfg = { .gen_gpio_num = m->pwm_gpio };
    ESP_RETURN_ON_ERROR(mcpwm_new_generator(m->oper, &gen_cfg, &m->gen),
                        TAG, "generator %d init failed", motor_idx);

    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_timer_event(m->gen,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                         MCPWM_TIMER_EVENT_EMPTY,
                                         MCPWM_GEN_ACTION_HIGH)),
        TAG, "gen timer action failed");

    ESP_RETURN_ON_ERROR(
        mcpwm_generator_set_action_on_compare_event(m->gen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                            m->cmpr,
                                            MCPWM_GEN_ACTION_LOW)),
        TAG, "gen compare action failed");

    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(m->timer), TAG, "timer enable failed");
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(m->timer, MCPWM_TIMER_START_NO_STOP),
                        TAG, "timer start failed");
    return ret;
}

/* ------------------------------------------------------------------ */

esp_err_t l298_motor_init(const l298_config_t *config, l298_handle_t *handle)
{
    ESP_RETURN_ON_FALSE(config && handle, ESP_ERR_INVALID_ARG, TAG, "null arg");
    ESP_RETURN_ON_FALSE(config->pwm_freq_hz > 0 && config->pwm_freq_hz <= 50000,
                        ESP_ERR_INVALID_ARG, TAG, "freq out of range (1-50000 Hz)");

    struct l298_dev_s *dev = calloc(1, sizeof(*dev));
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "no memory");

    dev->pwm_freq_hz = config->pwm_freq_hz;

    for (int i = 0; i < L298_MOTOR_MAX; i++) {
        l298_motor_priv_t         *m   = &dev->motors[i];
        const l298_motor_config_t *cfg = &config->motors[i];

        m->pwm_gpio  = cfg->pwm_gpio;
        m->dir1_gpio = cfg->dir1_gpio;
        m->dir2_gpio = cfg->dir2_gpio;

        esp_err_t err;
        err = gpio_out_init(cfg->dir1_gpio);
        if (err != ESP_OK) { free(dev); return err; }
        err = gpio_out_init(cfg->dir2_gpio);
        if (err != ESP_OK) { free(dev); return err; }

        gpio_set_level(cfg->dir1_gpio, 0);
        gpio_set_level(cfg->dir2_gpio, 0);

        err = motor_mcpwm_init(m, i, config->pwm_freq_hz);
        if (err != ESP_OK) { free(dev); return err; }

        ESP_LOGI(TAG, "Motor %c  PWM=GPIO%d  IN1=GPIO%d  IN2=GPIO%d  group=%d  freq=%lu Hz",
                 'A' + i, cfg->pwm_gpio, cfg->dir1_gpio, cfg->dir2_gpio,
                 (i < 2) ? 0 : 1, (unsigned long)config->pwm_freq_hz);
    }

    *handle = dev;
    return ESP_OK;
}

esp_err_t l298_motor_set(l298_handle_t handle, l298_motor_id_t motor_id,
                          l298_direction_t direction, uint8_t duty_pct)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "null handle");
    ESP_RETURN_ON_FALSE(motor_id < L298_MOTOR_MAX, ESP_ERR_INVALID_ARG,
                        TAG, "invalid motor id");

    l298_motor_priv_t *m = &handle->motors[motor_id];

    uint8_t eff = duty_pct;
    if (direction == L298_BRAKE || direction == L298_COAST) eff = 0;
    if (eff > 100) eff = 100;

    apply_direction(m, direction);

    uint32_t ticks = pct_to_ticks(m->period_ticks, eff);
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(m->cmpr, ticks),
                        TAG, "set compare failed");

    ESP_LOGD(TAG, "Motor %c  dir=%d  duty=%u%%", 'A' + motor_id, direction, eff);
    return ESP_OK;
}

esp_err_t l298_motor_stop(l298_handle_t handle, l298_motor_id_t motor_id)
{
    return l298_motor_set(handle, motor_id, L298_COAST, 0);
}

esp_err_t l298_motor_stop_all(l298_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "null handle");
    esp_err_t ret = ESP_OK;
    for (int i = 0; i < L298_MOTOR_MAX; i++) {
        esp_err_t e = l298_motor_set(handle, (l298_motor_id_t)i, L298_BRAKE, 0);
        if (e != ESP_OK) ret = e;
    }
    return ret;
}

esp_err_t l298_motor_deinit(l298_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "null handle");
    l298_motor_stop_all(handle);
    for (int i = 0; i < L298_MOTOR_MAX; i++) {
        l298_motor_priv_t *m = &handle->motors[i];
        if (m->timer) {
            mcpwm_timer_start_stop(m->timer, MCPWM_TIMER_STOP_EMPTY);
            mcpwm_timer_disable(m->timer);
            mcpwm_del_timer(m->timer);
        }
        if (m->gen)  mcpwm_del_generator(m->gen);
        if (m->cmpr) mcpwm_del_comparator(m->cmpr);
        if (m->oper) mcpwm_del_operator(m->oper);
    }
    free(handle);
    ESP_LOGI(TAG, "L298 component de-initialised");
    return ESP_OK;
}
