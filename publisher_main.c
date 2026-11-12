#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_nan.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "i2cdev.h"
#include "mpu6050_kalman.h"
#include "buzzer.h"
#include "cJSON.h"
#include "encoder_speed.h"
#include "motor_control.h"
#include "map.h"
#include "math.h"
#include "ssd1306_component.h"
#include "shared_data.h"
#include "kinematics.h"
#include "pid_ctrl.h"
#include "driver/mcpwm_prelude.h"
#include "bdc_motor.h"
#include "bdc_motor_interface.h"
/* ── Config ── */
//#define WIFI_SSID "Lab-Electronica2"
//#define WIFI_SSID      "IoT"
//#define WIFI_SSID      "vivo"
//#define WIFI_SSID      "ITCH-Edif-EE"
//#define WIFI_SSID      "vivo"
#define WIFI_SSID       "IZZI-9A06" 
//#define WIFI_SSID       "OPPO_10" 
//#define WIFI_PASS      "i9xb4arz"
#define WIFI_PASS      "DCA633E29A06"
//#define WIFI_PASS      "97843794"
//#define WIFI_PASS      "123456789"
//#define WIFI_PASS      "9antera5"
//#define WIFI_PASS "soyelectronico2"
//#define MQTT_BROKER    "mqtt://172.16.6.125"   // your broker IP
//#define MQTT_BROKER      "mqtt://172.16.6.132"   // your broker IP
#define MQTT_BROKER      "mqtt://192.168.0.10"   // your broker IP
//#define MQTT_BROKER    "mqtt://172.16.6.125"   // your broker IP
//#define MQTT_BROKER    "mqtt://172.16.6.131"   // your broker IP
//#define MQTT_BROKER    "mqtt://172.16.6.141"   // your broker IP 192.168.98.48
#define PUB_TOPIC      "var1"
#define PUB_TOPIC_2    "var2"
#define SUB_TOPIC      "omnibot/velocidades"
#define SUB_TOPIC_2    "omnibot/trayectoria"
#define SUB_TOPIC_CMD  "omnibot/cmd"
#define PUB_INTERVAL_S 0.5
#define LED_GPIO       GPIO_NUM_2
#define SAMPLE_MS 20
#define WHEEL_RADIUS 0.06 


static bool mqtt_connected = false;
static const char *TAG_OLED = "hello_oled";
static const char *TAG_MQTT = "MQTT_EXAMPLE";
static const char *APP_TAG = "SSD1306_DEMO";
static const char *TAG_CJSON = "Componente_cjson";
static l298_handle_t motor; 
static SemaphoreHandle_t motor_mutex; // Mutex para proteger el acceso a los motores


static esp_mqtt_client_handle_t mqtt_client = NULL;
QueueHandle_t sensor_queue;

#define I2C_MASTER_SCL_IO 23
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000

//Declaraciones para el buzzer
#define BUZZER_GPIO         GPIO_NUM_33  
#define BUZZER_LEDC_TIMER   LEDC_TIMER_0
#define BUZZER_LEDC_CHANNEL LEDC_CHANNEL_0
#define BUZZER_DEFAULT_FREQ 2000         

//valores entrada cinematica inversa funcion. 
typedef struct {
    float vx;       // body-frame velocity X  (m/s)
    float vy;       // body-frame velocity Y  (m/s)
    float wz;       // angular velocity       (rad/s)
} BodyVel;

//valores cinematica directa basado en valores de cinematica inversa. 
typedef struct {
    float w1;       // wheel 1 angular speed  (rad/s)
    float w2;       // wheel 2 angular speed  (rad/s)
    float w3;       // wheel 3 angular speed  (rad/s)
} WheelSpeeds;

typedef struct {
    BodyVel     body_vel;
    WheelSpeeds wheels;
    double      timestamp;  
} RobotCommand;

typedef enum {
    CMD_STOP = 0,
    CMD_FORWARD,
    CMD_BACKWARD,
    CMD_LEFT,
    CMD_RIGHT,
    CMD_CLOCKWISE,
    CMD_COUNTERCLOCKWISE,
} robot_cmd_t;

static volatile robot_cmd_t current_cmd = CMD_STOP;
static SemaphoreHandle_t cmd_mutex;   

typedef struct {
    l298_direction_t d[3];
    uint8_t          duty;
} cmd_entry_t;

static const cmd_entry_t CMD_TABLE[] = {
    [CMD_STOP]              = { {L298_COAST,    L298_COAST,    L298_COAST   }, 0  },
    [CMD_FORWARD]           = { {L298_BACKWARD, L298_FORWARD,  L298_COAST   }, 55 },
    [CMD_BACKWARD]          = { {L298_FORWARD,  L298_BACKWARD, L298_COAST   }, 55 },
    [CMD_LEFT]              = { {L298_FORWARD,  L298_FORWARD,  L298_BACKWARD}, 55 },
    [CMD_RIGHT]             = { {L298_BACKWARD, L298_BACKWARD, L298_FORWARD }, 55 },
    [CMD_CLOCKWISE]         = { {L298_FORWARD,  L298_FORWARD,  L298_FORWARD }, 50 },
    [CMD_COUNTERCLOCKWISE]  = { {L298_BACKWARD, L298_BACKWARD, L298_BACKWARD}, 50 },
};


/* ================================================================
   JSON COMMAND PARSER  (replaces handle_json_command for cmd topic)
   ================================================================ */
static void handle_cmd_message(const char *json_str)
{
    cJSON *root = cJSON_Parse(json_str);
    if (!root) {
        ESP_LOGE(TAG_CJSON, "CMD: JSON parse error");
        return;
    }

    cJSON *cmd_item = cJSON_GetObjectItemCaseSensitive(root, "cmd");
    if (!cJSON_IsString(cmd_item)) {
        ESP_LOGE(TAG_CJSON, "CMD: missing 'cmd' string field");
        cJSON_Delete(root);
        return;
    }

    const char *cmd_str = cmd_item->valuestring;
    robot_cmd_t new_cmd = CMD_STOP;

    if      (strcmp(cmd_str, "forward")          == 0) new_cmd = CMD_FORWARD;
    else if (strcmp(cmd_str, "backward")         == 0) new_cmd = CMD_BACKWARD;
    else if (strcmp(cmd_str, "left")             == 0) new_cmd = CMD_LEFT;
    else if (strcmp(cmd_str, "right")            == 0) new_cmd = CMD_RIGHT;
    else if (strcmp(cmd_str, "clockwise")        == 0) new_cmd = CMD_CLOCKWISE;
    else if (strcmp(cmd_str, "counterclockwise") == 0) new_cmd = CMD_COUNTERCLOCKWISE;
    else if (strcmp(cmd_str, "stop")             == 0) new_cmd = CMD_STOP;
    else {
        ESP_LOGW(TAG_CJSON, "CMD: unknown command '%s'", cmd_str);
    }

    if (xSemaphoreTake(cmd_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        current_cmd = new_cmd;
        xSemaphoreGive(cmd_mutex);
    }

    ESP_LOGI(TAG_CJSON, "CMD: '%s' → %d", cmd_str, (int)new_cmd);
    cJSON_Delete(root);
}



QueueHandle_t robot_cmd_queue; //Declaramos una cola que nos permita guardar valores recibidos del MQTT 



static void handle_json_command(cJSON *root)
{
    cJSON *vx = cJSON_GetObjectItemCaseSensitive(root, "vx");
    cJSON *vy = cJSON_GetObjectItemCaseSensitive(root, "vy");
    cJSON *wz = cJSON_GetObjectItemCaseSensitive(root, "wz");

    if (!cJSON_IsNumber(vx) || !cJSON_IsNumber(vy) || !cJSON_IsNumber(wz)) {
        ESP_LOGE(TAG_CJSON, "Missing or invalid vx/vy/wz fields");
        return;
    }

    float body_vx = (float)vx->valuedouble;
    float body_vy = (float)vy->valuedouble;
    float body_wz = (float)wz->valuedouble;

    float wheel_w1 = 0, wheel_w2 = 0, wheel_w3 = 0;
    cJSON *wheels = cJSON_GetObjectItemCaseSensitive(root, "wheels");
    if (cJSON_IsObject(wheels)) {
        cJSON *w1 = cJSON_GetObjectItemCaseSensitive(wheels, "w1");
        cJSON *w2 = cJSON_GetObjectItemCaseSensitive(wheels, "w2");
        cJSON *w3 = cJSON_GetObjectItemCaseSensitive(wheels, "w3");
        if (cJSON_IsNumber(w1)) wheel_w1 = (float)w1->valuedouble;
        if (cJSON_IsNumber(w2)) wheel_w2 = (float)w2->valuedouble;
        if (cJSON_IsNumber(w3)) wheel_w3 = (float)w3->valuedouble;
    }

    // ── read first so we preserve measured_rad_s, distance, trayectoria_id ──
    robot_state_t state;
    shared_data_read(&state);

    state.vx       = body_vx;
    state.vy       = body_vy;
    state.wz       = body_wz;
    state.wheels.w1 = wheel_w1;
    state.wheels.w2 = wheel_w2;
    state.wheels.w3 = wheel_w3;
    // measured_rad_s, distance, trayectoria_id are NOT touched

    shared_data_write(&state);

    ESP_LOGI(TAG_CJSON, "Vel → vx=%.3f vy=%.3f wz=%.3f", body_vx, body_vy, body_wz);
}


encoder_config_t cfg1 = {
    .gpio_a = GPIO_NUM_5,
    .gpio_b = GPIO_NUM_4,
    .pulses_per_rev = 1000,
    .sample_period_ms = 20.0f,
    .motor_id = 0,
};

encoder_config_t cfg2 = {
    .gpio_a = GPIO_NUM_18,
    .gpio_b = GPIO_NUM_19,
    .pulses_per_rev = 1000,
    .sample_period_ms = 20.0f,
    .motor_id = 1,
};

encoder_config_t cfg3 = {
    .gpio_a = GPIO_NUM_14,
    .gpio_b = GPIO_NUM_15,
    .pulses_per_rev = 1000,
    .sample_period_ms = 20.0f,
    .motor_id = 2,
};

//necesito pasar los parametros de la configuracion como una direcccion, es por eso 
//que ahgo un cast del tipo encoder_config
void shared_data_reset_distance(void)
{
    robot_state_t local;
    shared_data_read(&local);
    local.distance[0] = 0.0f;
    local.distance[1] = 0.0f;
    local.distance[2] = 0.0f;
    shared_data_write(&local);
}


void encoder_tarea (void*arg)
{
    //basicamente le digo convierte los datos que me enviaste al tipo encoder_confi_t
    //y guarda su direccion en un puntero tipo *cfg
    encoder_config_t *cfg = (encoder_config_t*)arg; 
    int motor_id = cfg->motor_id; //extraemos el id del motor para saber a que indice de la estructura de datos compartidos tenemos que escribir. 
    //recordar que recibimos la retroalimentacion de la velocidad en dato tipo float de tres elementos, es por ello que necesitamos el id. 
    encoder_handle_t enc; //creo un dato tipo encoder_handle_t
    ESP_ERROR_CHECK(encoder_init(cfg,&enc));
    ESP_ERROR_CHECK(encoder_start(enc)); 
    float rpm = 0.0f, rad_s = 0.0f;
    

    
    int64_t count = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(cfg->sample_period_ms));
        encoder_get_speed(enc, &rpm, &rad_s);
        //encoder_get_count(enc, &count);
        robot_state_t local; //creo un dato del tipo robot_state_t para guardar los datos en la estructura y pasarla por referencia a la funcion que utiliza el mutexz. 
        shared_data_read(&local);
        //escribimos en la estructura de datos compartida. 
        local.measured_rad_s[motor_id] = rad_s;
        local.distance[motor_id] += fabsf(rad_s) * WHEEL_RADIUS * (cfg->sample_period_ms / 1000.0f); // distancia = velocidad * tiempo

        shared_data_write(&local);
        //ESP_LOGI("motor_task", "Motor on GPIO %d/%d: count=%" PRId64
                // "  speed=%.1f RPM  (%.3f rad/s) distance=%.3f m",
                //cfg->gpio_a, cfg->gpio_b, count, rpm, rad_s, local.distance[motor_id]);
    }
}

l298_config_t cfg = {
        .pwm_freq_hz = 1000,   
        .motors = {
            [L298_MOTOR_A] = { .pwm_gpio=35, .dir1_gpio=12, .dir2_gpio=13 }, /* chip 1 */
            [L298_MOTOR_B] = { .pwm_gpio=32, .dir1_gpio=27, .dir2_gpio=26 }, /* chip 1 */
            [L298_MOTOR_C] = { .pwm_gpio=34, .dir1_gpio=25, .dir2_gpio=22 }, /* chip 2 */
        },
    };

void robot_control_task(void *arg)
{
     while (1) {
        vTaskDelay(pdMS_TO_TICKS(20));

        // Only run if trajectory task isn't holding the motors
        if (xSemaphoreTake(motor_mutex, 0) == pdFALSE) {
            continue;
        }

        robot_cmd_t cmd = CMD_STOP;
        if (xSemaphoreTake(cmd_mutex, 0) == pdTRUE) {
            cmd = current_cmd;
            xSemaphoreGive(cmd_mutex);
        }

        const cmd_entry_t *e = &CMD_TABLE[cmd];
        for (int i = 0; i < 3; i++) {
            l298_motor_set(motor, i, e->d[i], e->duty);
        }

        xSemaphoreGive(motor_mutex);
    }
}



//setpoints de velocidad para trayectorias. 
#define MOTOR0_SPEED_SETPOINT  9.5f
#define MOTOR1_SPEED_SETPOINT  6.0f   
#define MOTOR2_SPEED_SETPOINT  10.0f

//filtro para el encoder 
#define ENCODER_DEADBAND_RAD_S  0.3f


static pid_ctrl_block_handle_t tray_pid[3];

static void tray_pid_init(void)
{
    // ── motor 0 tuning ────────────────────────────────────────────
    pid_ctrl_config_t pid_m0 = {
        .init_param = {
            .kp           =  5.5f,   
            .ki           =  1.0f,
            .kd           =  0.01f,
            .max_output   =  100.0f,
            .min_output   =  0.0f,
            .max_integral =  50.0f,
            .min_integral = -50.0f,
            .cal_type     = PID_CAL_TYPE_POSITIONAL,
        },
    };

    // ── motor 1 tuning ────────────────────────────────────────────
    pid_ctrl_config_t pid_m1 = {
        .init_param = {
            .kp           =  1.5f,    
            .ki           =  1.0f,
            .kd           =  0.01f,
            .max_output   =  100.0f,
            .min_output   =  0.0f,
            .max_integral =  50.0f,
            .min_integral = -50.0f,
            .cal_type     = PID_CAL_TYPE_POSITIONAL,
        },
    };

    
    pid_ctrl_config_t pid_m2 = pid_m1;

    pid_ctrl_config_t *cfgs[3] = {&pid_m0, &pid_m1, &pid_m2};
    for (int i = 0; i < 3; i++)
        ESP_ERROR_CHECK(pid_new_control_block(cfgs[i], &tray_pid[i]));
}

static void tray_pid_reset(void)
{
    for (int i = 0; i < 3; i++)
        pid_reset_ctrl_block(tray_pid[i]);
}
//====================================================================================================
// FUNCION QUE ME PERMITE UTILIZAR LA ODOMETRIA PARA AVANZAR CIERTA DISTANCIA
//====================================================================================================

static void drive_until_pid(float target_m,
                             bool m0_forward, bool m1_forward,
                             bool active[3])
{
    const float setpoints[3] = {
        MOTOR0_SPEED_SETPOINT,
        MOTOR1_SPEED_SETPOINT,
        MOTOR2_SPEED_SETPOINT,
    };

    robot_state_t local;
    shared_data_read(&local);

    
    float start[3] = {0};
    for (int i = 0; i < 3; i++)
        start[i] = active[i] ? local.distance[i] : 0.0f;

    //tray_pid_reset();

    while (1) {
        shared_data_read(&local);

      
        float driven_sum   = 0.0f;
        int   active_count = 0;
        float driven_each[3] = {0};

        for (int i = 0; i < 3; i++) {
            if (active[i]) {
                driven_each[i]  = local.distance[i] - start[i];
                driven_sum     += driven_each[i];
                active_count++;
            }
        }
        float driven_avg = (active_count > 0) ? (driven_sum / active_count) : 0.0f;

        ESP_LOGI("TRAY", "avg=%.3f/%.3f  m0=%.3f  m1=%.3f",
                 driven_avg, target_m, driven_each[0], driven_each[1]);

        if (driven_avg >= target_m) break;

        
        for (int i = 0; i < 3; i++) {
            if (!active[i]) {
                l298_motor_set(motor, i, L298_COAST, 0);
                continue;
            }

            
            float measured = fabsf(local.measured_rad_s[i]);
            if (measured < ENCODER_DEADBAND_RAD_S) measured = 0.0f;

            
            float sync_gain  = 3.0f;  // ← tune this
            float sync_error = driven_avg - driven_each[i];  

            float speed_error = setpoints[i] - measured;
            float total_error = speed_error + (sync_error * sync_gain);

            float output = 0.0f;
            ESP_ERROR_CHECK(pid_compute(tray_pid[i], total_error, &output));

            uint8_t duty = (uint8_t)fminf(fmaxf(output, 0.0f), 100.0f);

            l298_direction_t dir;
            if      (i == 0) dir = m0_forward ? L298_FORWARD : L298_BACKWARD;
            else if (i == 1) dir = m1_forward ? L298_FORWARD : L298_BACKWARD;
            else             dir = L298_FORWARD;

            l298_motor_set(motor, i, dir, duty);

            ESP_LOGD("TRAY", "M%d: meas=%.2f set=%.2f sync=%.2f duty=%d",
                     i, measured, setpoints[i], sync_error, duty);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    for (int i = 0; i < 3; i++)
        l298_motor_set(motor, i, L298_COAST, 0);
}

//====================================================================================================
// FUNCION QUE ME PERMITE ACTIVAR EL GIRO POR DETERMINADO TIEMPO. 
//====================================================================================================

static void turn_pid(bool clockwise, uint32_t turn_ms)
{
    tray_pid_reset();
    uint32_t elapsed = 0;

    const float setpoints[3] = {
        MOTOR0_SPEED_SETPOINT,
        MOTOR1_SPEED_SETPOINT,
        MOTOR2_SPEED_SETPOINT,
    };

    // ── direction per motor for each turn direction ───────────────
    // adjust these based on your physical wheel placement
    l298_direction_t dir_cw[3]  = { L298_FORWARD,  L298_FORWARD,  L298_FORWARD  };
    l298_direction_t dir_ccw[3] = { L298_BACKWARD, L298_BACKWARD, L298_BACKWARD };

    while (elapsed < turn_ms) {
        robot_state_t local;
        shared_data_read(&local);

        for (int i = 0; i < 3; i++) {
            float error  = setpoints[i] - fabsf(local.measured_rad_s[i]);
            float output = 0.0f;
            ESP_ERROR_CHECK(pid_compute(tray_pid[i], error, &output));
            uint8_t duty = (uint8_t)fminf(fmaxf(output, 0.0f), 100.0f);

            l298_direction_t dir = clockwise ? dir_cw[i] : dir_ccw[i];
            l298_motor_set(motor, i, dir, duty);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
        elapsed += 20;
    }

    for (int i = 0; i < 3; i++)
        l298_motor_set(motor, i, L298_COAST, 0);
}


#define vuelta_90_grados 530
#define vuelta_180_grados 900

static void ejecutar_trayectoria_1(void)
{
    // motors 0 and 1 drive straight, motor 2 coasts
    bool active_straight[3] = {true, true, false};
    drive_until_pid(0.2f, false, true, active_straight);  // m0=BACKWARD, m1=FORWARD
    vTaskDelay(pdMS_TO_TICKS(300));

    turn_pid(true, vuelta_90_grados);   // all motors forward for 600ms → tune time for ~90°
    vTaskDelay(pdMS_TO_TICKS(300));

    drive_until_pid(0.1f, false, true, active_straight);
}

static void ejecutar_trayectoria_2(void) 
{

    bool active_straight[3] = {true, true, false};
    turn_pid(true, vuelta_180_grados);
    vTaskDelay(pdMS_TO_TICKS(300));
    drive_until_pid(0.1f, false, true, active_straight);
    vTaskDelay(pdMS_TO_TICKS(300));
    turn_pid(true, vuelta_90_grados);   

}
static void ejecutar_trayectoria_3(void) 
{

    bool active_straight[3] = {true, true, false};
    drive_until_pid(0.20f, false, true, active_straight);
    vTaskDelay(pdMS_TO_TICKS(300));
    

}

static void ejecutar_trayectoria_4(void) 
{

    bool active_straight[3] = {true, true, false};
    turn_pid(true, vuelta_180_grados);
    vTaskDelay(pdMS_TO_TICKS(300));
    drive_until_pid(0.1f, false, true, active_straight);
    vTaskDelay(pdMS_TO_TICKS(300));
    turn_pid(true, vuelta_90_grados);
}

static void ejecutar_trayectoria_5(void) 
{

    bool active_straight[3] = {true, true, false};
    drive_until_pid(0.1f, false, true, active_straight);
    vTaskDelay(pdMS_TO_TICKS(300));
    turn_pid(true, vuelta_90_grados);
    drive_until_pid(0.1f, false, true, active_straight);

}

static void ejecutar_trayectoria_6(void) 
{

    bool active_straight[3] = {true, true, false};
    turn_pid(true, 500);
    vTaskDelay(pdMS_TO_TICKS(300));
    drive_until_pid(0.1f, false, true, active_straight);
    vTaskDelay(pdMS_TO_TICKS(300));
    turn_pid(false, vuelta_90_grados);
}

static void ejecutar_trayectoria_7(void) 
{

    bool active_straight[3] = {true, true, false};
    drive_until_pid(0.1f, false, true, active_straight);
    vTaskDelay(pdMS_TO_TICKS(300));
    turn_pid(true, vuelta_90_grados);
    vTaskDelay(pdMS_TO_TICKS(300));
    drive_until_pid(0.30f, false, true, active_straight);
    vTaskDelay(pdMS_TO_TICKS(300));
    turn_pid(false, vuelta_180_grados);
}


//====================================================================================================
// FUNCION QUE SE ENCARGA DE ESCOGER LA TRAYECTORIA BASADO EN LA SOLICITUD DEL BROKER. 
//====================================================================================================

void trajectory_task(void *arg)
{
    tray_pid_init(); 

    while (1) {
        // ── block here until a trajectory is requested ────────
        vTaskDelay(pdMS_TO_TICKS(50));

        robot_state_t local;
        shared_data_read(&local);

        if (local.trayectoria_id == 0) {
            continue;  
        }

        int tray_id = local.trayectoria_id;
        //Utilizamos un mutex para evitar que las dos tareas accesen al mismo recurso en este caso los motores. 
        if (xSemaphoreTake(motor_mutex, portMAX_DELAY) == pdTRUE) {

            
            local.trayectoria_id = 0;
            shared_data_write(&local);

            ESP_LOGI("TRAY", "Ejecutando trayectoria %d", tray_id);

            if      (tray_id == 1) ejecutar_trayectoria_1();
            else if (tray_id == 2) ejecutar_trayectoria_2();
            else if (tray_id == 3) ejecutar_trayectoria_3();
            else if (tray_id == 4) ejecutar_trayectoria_4();
            else if (tray_id == 5) ejecutar_trayectoria_5();
            else if (tray_id == 6) ejecutar_trayectoria_6();
            else if (tray_id == 7) ejecutar_trayectoria_7();
            else {
                ESP_LOGW("TRAY", "Trayectoria %d no definida", tray_id);
            }

            //Detiene los motores cuando termina la rutina
            l298_motor_set(motor, 0, L298_COAST, 0);
            l298_motor_set(motor, 1, L298_COAST, 0);
            l298_motor_set(motor, 2, L298_COAST, 0);

            ESP_LOGI("TRAY", "Trayectoria %d completada", tray_id);

            xSemaphoreGive(motor_mutex);          }
    }
}


typedef struct {
    l298_config_t *motor_cfg; //Utilizamos un wraper para pasar la configuracion a la funcion de control de motores.  
    int trayectoria_id; 

}control_trayectoria_arg_t; 


/* ── SSD1306 TASK ───────────────────────────────────────── */
void ssd1306_task(void *pvParameters)
{
    int valor_recibido = (int)pvParameters; //le aplicamos un cast. 
    ssd1306_t dev;

    /* Inicializar pantalla */
    if (ssd1306_init(&dev,
                     I2C_MASTER_NUM,
                     I2C_MASTER_SDA_IO,
                     I2C_MASTER_SCL_IO,
                     0x3C) != ESP_OK)
    {
        //ESP_LOGE(TAG, "SSD1306 init failed");
        vTaskDelete(NULL);
    }

    int counter = 0;

    if(valor_recibido == 1){
        /* Limpiar framebuffer */
        ssd1306_clear(&dev);

        /* Dibujar texto */
        ssd1306_draw_string(&dev, 0, 0, "Conexion Exitosa");
        ssd1306_flush(&dev);
        vTaskDelay(pdMS_TO_TICKS(5000));
        ssd1306_clear(&dev);
        ssd1306_display_on(&dev,1);
        ssd1306_flush(&dev);
        vTaskDelay(pdMS_TO_TICKS(1000)); 

        
    }
    
    robot_state_t local; 
    char line[32]; 
   
     while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        shared_data_read(&local);
        /* Limpiar framebuffer */
        ssd1306_clear(&dev);

        /* Dibujar texto */
        //ssd1306_draw_string(&dev, 0, 0, "ESP32 SSD1306");
        
        //char buffer[32];
        //snprintf(buffer, sizeof(buffer), "Count: %d", counter++);
        //ssd1306_draw_string(&dev, 0, 16, buffer);

        /* Dibujar algo grafico */
        //ssd1306_draw_rect(&dev, 0, 32, 50, 20, true);

        /* Enviar a pantalla */
        //if (ssd1306_flush(&dev) != ESP_OK)
        //{
            
            //ESP_LOG(TAG, "Display update failed");
        //}
        //vTaskDelay(pdMS_TO_TICKS(1000));
        //vTaskDelete(NULL); //Eliminamos la tarea. 
    //}
        // Row 0 — vx and vy
        snprintf(line, sizeof(line), "vx=%.2f vy=%.2f", local.vx, local.vy);
        ssd1306_draw_string(&dev, 0, 0, line);

        // Row 1 — angular velocity
        snprintf(line, sizeof(line), "wz=%.3f rad/s", local.wz);
        ssd1306_draw_string(&dev, 0, 16, line);

        // Row 2 — wheel speeds w1 w2
        snprintf(line, sizeof(line), "w1=%.1f w2=%.1f", local.wheels.w1, local.wheels.w2);
        ssd1306_draw_string(&dev, 0, 32, line);

        // Row 3 — wheel speed w3
        snprintf(line, sizeof(line), "w3=%.2f", local.wheels.w3);
        ssd1306_draw_string(&dev, 0, 48, line);

        ssd1306_flush(&dev);
    }
}


/* ─────────────────────────────────────────────
   MQTT Event Handler
   ───────────────────────────────────────────── */
static void mqtt_event_handler(void *arg,
                                esp_event_base_t base,
                                int32_t event_id,
                                void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {

    case MQTT_EVENT_BEFORE_CONNECT:
        ESP_LOGI(TAG_MQTT, "Before connect");
        break;

    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG_MQTT, "Connected to broker");
        mqtt_connected = true;
        // Subscribe as soon as we connect
        int msg_id = esp_mqtt_client_subscribe(mqtt_client, SUB_TOPIC, 1);
        int msg_id2 = esp_mqtt_client_subscribe(mqtt_client, SUB_TOPIC_2, 1);
        int msg_id3 = esp_mqtt_client_subscribe(mqtt_client, SUB_TOPIC_CMD, 1);
        ESP_LOGI(TAG_MQTT, "Subscribed to '%s', msg_id=%d", SUB_TOPIC, msg_id);
        ESP_LOGI(TAG_MQTT, "Subscribed to '%s', msg_id=%d", SUB_TOPIC_2, msg_id2);
        ESP_LOGI(TAG_MQTT, "Subscribed to '%s' id=%d", SUB_TOPIC_CMD, msg_id3);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG_MQTT, "Disconnected – will auto-reconnect");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG_MQTT, "Subscribe ACK, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG_MQTT, "Unsubscribe ACK, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG_MQTT, "Publish ACK, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_DATA: {
    ESP_LOGI(TAG_MQTT, "Topic: %.*s", event->topic_len, event->topic);

    char *payload = strndup(event->data, event->data_len);
    if (!payload) break;

    if (strncmp(event->topic, SUB_TOPIC_CMD, event->topic_len) == 0) {
        handle_cmd_message(payload);       // passes raw string, function parses it
        free(payload);
    } else {
        cJSON *root = cJSON_Parse(payload);
        free(payload);                     // safe to free now, cJSON has its own copy

        if (!root) { ESP_LOGE(TAG_MQTT, "JSON parse error"); break; }

        if (strncmp(event->topic, SUB_TOPIC_2, event->topic_len) == 0) {
            cJSON *id = cJSON_GetObjectItemCaseSensitive(root, "trayectoria_id");
            if (cJSON_IsNumber(id)) {
                robot_state_t local;
                shared_data_read(&local);
                local.trayectoria_id = id->valueint;
                shared_data_write(&local);
                ESP_LOGI(TAG_MQTT, "Trayectoria recibida: %d", id->valueint);
            }
        } else {
            handle_json_command(root);
        }
        cJSON_Delete(root);
    }
    break;
    }

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG_MQTT, "MQTT error");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGE(TAG_MQTT, "Transport error: %s",
                     strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;

    default:
        break;
    }
}

/* ─────────────────────────────────────────────
   MQTT Init
   ───────────────────────────────────────────── */
static void mqtt_app_start(void)
{
//inicializacion del componente del buzzer 
     buzzer_config_t buzzer_cfg = {
        .gpio_num   = BUZZER_GPIO,
        .timer_num  = BUZZER_LEDC_TIMER,
        .channel    = BUZZER_LEDC_CHANNEL,
        .freq_hz    = BUZZER_DEFAULT_FREQ,
    };
    ESP_ERROR_CHECK(buzzer_init(&buzzer_cfg));
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER,
        // Optional auth:
         .credentials.username = "manuel",
         .credentials.authentication.password = "1234",
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client,
                                   ESP_EVENT_ANY_ID,
                                   mqtt_event_handler,
                                   NULL);
    esp_mqtt_client_start(mqtt_client);
    
    xTaskCreate(ssd1306_task,"ssd1306_task",4096,(void*)1,1,NULL);
    ESP_ERROR_CHECK(buzzer_beep_pattern(5, 4000, 80, 80));
    
    //xTaskCreate(ssd1306_task,"ssd1306_task",4096,NULL,5,NULL);
     
        
}

/* ─────────────────────────────────────────────
   Publisher Task  (runs every PUB_INTERVAL_S s)
   ───────────────────────────────────────────── */
static void publish_task(void *pvParameters)
{
    char payload_yaw[64];
    char payload_pitch[64]; 
    int  count = 0;
    sensor_data_t received; 

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(PUB_INTERVAL_S * 1000));

            //if (xQueueReceive(sensor_queue, &received, 0) == pdTRUE) {
                //ESP_LOGI(TAG, "Received from queue - Roll: %+7.2f°  Pitch: %+7.2f°  Yaw: %+7.2f°",
                        //received.roll, received.pitch, received.yaw);
            //}
            if (xQueueReceive(sensor_queue, &received, portMAX_DELAY) == pdTRUE) {
                ESP_LOGI(TAG_MQTT, "Received from queue - Roll: %+7.2f°  Pitch: %+7.2f°  Yaw: %+7.2f°",
                        received.roll, received.pitch, received.yaw);
            }
 
        snprintf(payload_yaw, sizeof(payload_yaw),
                 "%.2f", received.yaw);
                 //count++;
                 //sensor_data_t received = {0};
        snprintf(payload_pitch, sizeof(payload_pitch),
                 "%.2f", received.pitch);


        int msg_id = esp_mqtt_client_publish(mqtt_client,
                                             PUB_TOPIC,
                                             payload_yaw,
                                             0,    // len 0 = auto strlen
                                             1,    // QoS 1
                                             0);   // retain = false
                                             
        int msg_id_2 = esp_mqtt_client_publish(mqtt_client,
                                             PUB_TOPIC_2,
                                             payload_pitch,
                                             0,    // len 0 = auto strlen
                                             1,    // QoS 1
                                             0);   // retain = false
        ESP_LOGI(TAG_MQTT, "Published '%s' → msg_id=%d", payload_pitch, msg_id);






    }
}
//====================================================================================================
// FUNCION QUE SE ENCARGA DEL wifi
//====================================================================================================
static void wifi_event_handler(void *arg,
                                esp_event_base_t base,
                                int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
    {   ESP_LOGI(TAG_MQTT, "WiFi started, connecting...");
        esp_wifi_connect();
        
    }
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
    {   ESP_LOGW(TAG_MQTT, "WiFi disconnected, retrying...");
        esp_wifi_connect();
    }
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG_MQTT, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
        mqtt_app_start();                        // start MQTT once connected
        xTaskCreate(publish_task, "pub_task",
                    4096, NULL, 5, NULL);        // start publisher task
    }
}

static void wifi_init_sta(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        wifi_event_handler, NULL, NULL);

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    esp_wifi_start();
}

/* ─────────────────────────────────────────────
   Entry point
   ───────────────────────────────────────────── */
void app_main(void)
{
    nvs_flash_init();
    shared_data_init(); 
    sensor_queue = xQueueCreate(10, sizeof(sensor_data_t));//cola para compartir datos entre tareas y publicar al mqtt. 
    //robot_cmd_queue = xQueueCreate(1, sizeof(RobotCommand));//Cola para enviar los datos que recibimos por json. 
    //configASSERT(robot_cmd_queue != NULL);

    ESP_ERROR_CHECK(i2cdev_init());
    ESP_ERROR_CHECK(l298_motor_init(&cfg, &motor));
    motor_mutex = xSemaphoreCreateMutex();
    configASSERT(motor_mutex != NULL);

    cmd_mutex = xSemaphoreCreateMutex();        
    configASSERT(cmd_mutex != NULL);            

    wifi_init_sta();
    xTaskCreate(robot_control_task,"robot_ctrl_task",8192,NULL,4,NULL); //Funcion que recibe los datos de la cola. 
    xTaskCreate(trajectory_task,    "tray_task", 8192, NULL, 3, NULL);  // lower priority
    //xTaskCreate(control_trayectoria_task,"control_trayectoria_task",8192,&cfg,4,NULL);
    //xTaskCreate(ssd1306_task_mpu,"ssd1306_task_mpu",4096,NULL,1,NULL); 
    //xTaskCreate(mpu6050_kalman_task, "mpu6050_kalman",
                //configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
    mpu6050_kalman_start(); // This will start the task that reads the sensor and sends data to the queue
    //xTaskCreate(ssd1306_task,"ssd1306_task",4096,NULL,5,NULL);
    xTaskCreate(encoder_tarea, "motor1", 4096, &cfg1, 5, NULL);
    xTaskCreate(encoder_tarea, "motor2", 4096, &cfg2, 5, NULL);
    xTaskCreate(encoder_tarea, "motor3", 4096, &cfg3, 5, NULL);
    
}
    












