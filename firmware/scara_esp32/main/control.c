/*
 * control.c — Tarea de control a 1 kHz en el core 1.
 *
 * Por qué aquí y no en ROS: el viaje ida y vuelta por USB+DDS tarda
 * varios ms y con jitter; un PID con retardo variable se desestabiliza. Además el
 * enclavamiento con finales y el watchdog tienen que reaccionar en ~1 ms.
 *
 * Este archivo solo toca hardware; TODA la lógica (modos, PID, enclavamiento, watchdog,
 * bloqueo) está en ctrl_logic.c, que se prueba en el PC y la usa el simulador.
 *
 * Cada milisegundo:
 *   1. procesa los comandos que llegaron (cola desde el core 0)
 *   2. lee finales (debounce) y encoders
 *   3. ctrl_step() → duty por junta
 *   4. aplica a los motores y deja una foto del estado para micro-ROS
 */
#include "control.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "config.h"
#include "encoder.h"
#include "limit_switches.h"
#include "motor.h"

_Static_assert(NUM_JOINTS == CTRL_NUM_JOINTS, "pins.h y ctrl_config.h no coinciden");

static const char *TAG = "control";

typedef struct {
    int32_t mode, v1, v2, v3;
} cmd_msg_t;

static QueueHandle_t s_queue;
static ctrl_state_t s_ctrl;

/* Foto del estado (la lee la tarea micro-ROS en el otro núcleo). */
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static struct {
    int32_t counts[NUM_JOINTS];
    uint8_t flags;
} s_status;

static void control_task(void *arg)
{
    (void)arg;
    int32_t pos[NUM_JOINTS];
    int32_t out[NUM_JOINTS];
    uint8_t prev_flags = 0;

    for (int j = 0; j < NUM_JOINTS; j++) pos[j] = encoder_get(j);
    ctrl_init(&s_ctrl, pos);

    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));

        /* 1. Comandos pendientes (se vacía la cola: no se pierde un ZERO ni una config) */
        cmd_msg_t c;
        while (xQueueReceive(s_queue, &c, 0) == pdTRUE) {
            const ctrl_effect_t eff = ctrl_command(&s_ctrl, c.mode, c.v1, c.v2, c.v3);
            if (eff.zero) encoder_set(eff.joint, eff.value);
        }

        /* 2. Sensores */
        limits_update();
        const uint8_t lim = limits_get();
        for (int j = 0; j < NUM_JOINTS; j++) pos[j] = encoder_get(j);

        /* 3. Lógica */
        const uint8_t flags = ctrl_step(&s_ctrl, pos, lim, out);

        /* 4. Motores (0 = freno activo) */
        for (int j = 0; j < NUM_JOINTS; j++) {
            if (out[j] == 0) motor_brake(j);
            else motor_set(j, out[j]);
        }

        if ((flags & STATUS_BIT_STALL) && !(prev_flags & STATUS_BIT_STALL))
            ESP_LOGW(TAG, "bloqueo detectado: junta cortada");
        prev_flags = flags;

        portENTER_CRITICAL(&s_mux);
        memcpy(s_status.counts, pos, sizeof(pos));
        s_status.flags = flags;
        portEXIT_CRITICAL(&s_mux);
    }
}

esp_err_t control_start(void)
{
    s_queue = xQueueCreate(CMD_QUEUE_LEN, sizeof(cmd_msg_t));
    if (s_queue == NULL) return ESP_ERR_NO_MEM;
    motor_brake_all();
    BaseType_t ok = xTaskCreatePinnedToCore(control_task, "control", CONTROL_TASK_STACK, NULL,
                                            CONTROL_TASK_PRIO, NULL, CONTROL_TASK_CORE);
    ESP_LOGI(TAG, "tarea de control %s (core %d, %d ms)", ok == pdPASS ? "lanzada" : "FALLÓ",
             CONTROL_TASK_CORE, CONTROL_PERIOD_MS);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}

void control_command(int32_t mode, int32_t v1, int32_t v2, int32_t v3)
{
    const cmd_msg_t c = {mode, v1, v2, v3};
    /* Sin espera: si la cola se llenara (no debería a 50 Hz), se descarta el mensaje;
     * el siguiente keepalive lo repone. */
    (void)xQueueSend(s_queue, &c, 0);
}

void control_get_status(int32_t counts[NUM_JOINTS], uint8_t *limits_and_flags)
{
    portENTER_CRITICAL(&s_mux);
    memcpy(counts, s_status.counts, sizeof(s_status.counts));
    *limits_and_flags = s_status.flags;
    portEXIT_CRITICAL(&s_mux);
}
