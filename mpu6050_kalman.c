#include "mpu6050_kalman.h"
#include <math.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"

/* ─── Hardware config ─────────────────────────────────────────────────────── */
#define ADDR            MPU6050_I2C_ADDRESS_LOW
#define SCL_MICRO       GPIO_NUM_23
#define SDA_MICRO       GPIO_NUM_21

/* ─── Kalman filter sample period ────────────────────────────────────────── */
#define SAMPLE_PERIOD_MS  10   /* 100 Hz – keep in sync with vTaskDelay */

static const char *TAG = "mpu6050_kalman";



/* ══════════════════════════════════════════════════════════════════════════
 *  Kalman filter
 * ══════════════════════════════════════════════════════════════════════════ */

void kalman_init(kalman_t *k, float initial_angle)
{
    k->angle     = initial_angle;
    k->bias      = 0.0f;
    k->P[0][0]   = 0.0f;
    k->P[0][1]   = 0.0f;
    k->P[1][0]   = 0.0f;
    k->P[1][1]   = 0.0f;
    k->Q_angle   = 0.001f;
    k->Q_bias    = 0.003f;
    k->R_measure = 0.03f;
}

float kalman_update(kalman_t *k, float gyro_rate, float accel_angle, float dt)
{
    /* ── PREDICT ── */
    float rate = gyro_rate - k->bias;
    k->angle  += dt * rate;

    k->P[0][0] += dt * (dt * k->P[1][1] - k->P[0][1] - k->P[1][0] + k->Q_angle);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += k->Q_bias * dt;

    /* ── UPDATE ── */
    float S  = k->P[0][0] + k->R_measure;
    float K0 = k->P[0][0] / S;
    float K1 = k->P[1][0] / S;

    float y   = accel_angle - k->angle;
    k->angle += K0 * y;
    k->bias  += K1 * y;

    float P00_temp = k->P[0][0];
    float P01_temp = k->P[0][1];

    k->P[0][0] -= K0 * P00_temp;
    k->P[0][1] -= K0 * P01_temp;
    k->P[1][0] -= K1 * P00_temp;
    k->P[1][1] -= K1 * P01_temp;

    return k->angle;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Accelerometer angle helpers
 * ══════════════════════════════════════════════════════════════════════════ */

static inline float accel_roll(mpu6050_acceleration_t *a)
{
    return atan2f(a->y, a->z) * (180.0f / (float)M_PI);
}

static inline float accel_pitch(mpu6050_acceleration_t *a)
{
    return atan2f(-a->x, sqrtf(a->y * a->y + a->z * a->z)) * (180.0f / (float)M_PI);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Main FreeRTOS task
 * ══════════════════════════════════════════════════════════════════════════ */

void mpu6050_kalman_start(void)
{
    //ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(mpu6050_kalman_task,
                "mpu6050_kalman",
                configMINIMAL_STACK_SIZE * 8,
                NULL,
                5,
                NULL);
}

void mpu6050_kalman_task(void *pvParameters)
{
    mpu6050_dev_t dev = { 0 };

    ESP_ERROR_CHECK(mpu6050_init_desc(&dev, ADDR, 0, SDA_MICRO, SCL_MICRO));
    sensor_data_t giroscope; //instaciacion de la estructura con la que mandamos datos. 

    /* Wait until sensor responds */
    while (1) {
        esp_err_t res = i2c_dev_probe(&dev.i2c_dev, I2C_DEV_WRITE);
        if (res == ESP_OK) {
            ESP_LOGI(TAG, "MPU6050 found");
            break;
        }
        ESP_LOGE(TAG, "MPU6050 not found, retrying...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_ERROR_CHECK(mpu6050_init(&dev));
    ESP_LOGI(TAG, "Accel range: %d | Gyro range: %d",
             dev.ranges.accel, dev.ranges.gyro);

    /* Seed Kalman filters with first reading */
    mpu6050_acceleration_t accel = { 0 };
    mpu6050_rotation_t     rot   = { 0 };
    ESP_ERROR_CHECK(mpu6050_get_motion(&dev, &accel, &rot));

    kalman_t kf_roll, kf_pitch;
    kalman_init(&kf_roll,  accel_roll(&accel));
    kalman_init(&kf_pitch, accel_pitch(&accel));

    float   yaw       = 0.0f;
    int64_t last_time = esp_timer_get_time();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));

        int64_t now = esp_timer_get_time();
        float   dt  = (float)(now - last_time) * 1e-6f;
        last_time   = now;
        if (dt > 0.5f) dt = 0.5f;

        float temp;
        ESP_ERROR_CHECK(mpu6050_get_temperature(&dev, &temp));
        ESP_ERROR_CHECK(mpu6050_get_motion(&dev, &accel, &rot));

        float roll  = kalman_update(&kf_roll,  rot.x, accel_roll(&accel),  dt);
        float pitch = kalman_update(&kf_pitch, rot.y, accel_pitch(&accel), dt);
        
        yaw += rot.z * dt;
        if      (yaw >  180.0f) yaw -= 360.0f;
        else if (yaw < -180.0f) yaw += 360.0f;

        giroscope.roll = roll;
        giroscope.pitch = pitch;
        giroscope.yaw = yaw;

        xQueueSend(sensor_queue, &giroscope, portMAX_DELAY);

        ESP_LOGI(TAG, "Roll: %+7.2f°  Pitch: %+7.2f°  Yaw: %+7.2f°  Temp: %.1f°C",
                 roll, pitch, yaw, temp);
    }
}