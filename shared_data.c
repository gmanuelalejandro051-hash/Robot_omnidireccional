#include "shared_data.h"
#include "esp_log.h"

robot_state_t    g_robot_state  = {0};
SemaphoreHandle_t g_state_mutex = NULL;

void shared_data_init(void)
{
    g_state_mutex = xSemaphoreCreateMutex();
    configASSERT(g_state_mutex != NULL);
}

//Funcion que copia la estructura de robot_state_t a la variable global g_robot_state, utilizando un mutex para asegurar que no haya condiciones de carrera al acceder a los datos compartidos.
//Utilizamos un mutex para copiar la estructura y guardarla en g_robot_state. 
void shared_data_write(const robot_state_t *src)
{
    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_robot_state = *src;           // struct copy — very fast
        xSemaphoreGive(g_state_mutex);
    } else {
        ESP_LOGW("SHARED", "Write mutex timeout");
    }
}

void shared_data_read(robot_state_t *dst)
{
    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        *dst = g_robot_state;           // struct copy — very fast
        xSemaphoreGive(g_state_mutex);
    } else {
        ESP_LOGW("SHARED", "Read mutex timeout");
    }
}