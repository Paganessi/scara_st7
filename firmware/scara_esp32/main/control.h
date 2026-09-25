/*
 * control.h — Tarea de control a 1 kHz (core 1): el "pegamento" entre el hardware
 * (motores, encoders, finales) y la lógica pura de ctrl_logic.c.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "ctrl_logic.h"
#include "pins.h"

/* Lanza la tarea de control (motores/encoders/finales ya inicializados). Arranca en STOP. */
esp_err_t control_start(void);

/* Llamar al recibir /scara/cmd (desde la tarea micro-ROS, core 0). Encola el mensaje;
 * la tarea de control lo procesa en su siguiente ciclo (≤ 1 ms). Nunca bloquea. */
void control_command(int32_t mode, int32_t v1, int32_t v2, int32_t v3);

/* Foto del estado para publicar: cuentas de las 3 juntas y byte de finales+diagnóstico. */
void control_get_status(int32_t counts[NUM_JOINTS], uint8_t *limits_and_flags);
