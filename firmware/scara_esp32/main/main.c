/*
 * main.c — Firmware SCARA Estación 7 (ESP32 + ESP-IDF + micro-ROS).
 *
 * Orden de arranque (importa):
 *   1. motores en freno ANTES que nada (sin tirones al energizar)
 *   2. encoders y finales
 *   3. tarea de control 1 kHz en core 1 (arranca en STOP: sin comandos no se mueve nada)
 *   4. tarea micro-ROS en core 0 (espera al agente, reintenta si se cae)
 */
#include "control.h"
#include "encoder.h"
#include "limit_switches.h"
#include "motor.h"
#include "uros.h"

void app_main(void)
{
    ESP_ERROR_CHECK(motor_init());
    ESP_ERROR_CHECK(encoder_init());
    ESP_ERROR_CHECK(limits_init());
    ESP_ERROR_CHECK(control_start());
    ESP_ERROR_CHECK(uros_start());
}
