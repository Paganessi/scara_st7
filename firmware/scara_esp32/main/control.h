/*
 * control.h — Tarea de control a 1 kHz (core 1).
 *
 * Recibe comandos con el formato de /scara/cmd ([modo, v1, v2, v3])
 * y entrega el estado (cuentas + finales) para publicarlo.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "pins.h"

typedef enum {
    CTRL_MODE_STOP = 0,     /* freno en las 3 juntas */
    CTRL_MODE_POSITION = 1, /* v = metas en cuentas → PID */
    CTRL_MODE_VELOCITY = 2, /* v = duty con signo en ‰ → lazo abierto */
    CTRL_MODE_ZERO = 3,     /* v1 = junta 1..3, v2 = valor a cargar en su contador */
    /* Extensión: configuración en caliente desde ROS, sin reflashear.
     * No cambian el modo de movimiento; sí cuentan como keepalive. */
    CTRL_MODE_SET_GAINS = 4, /* v1 = junta 1..3, v2 = KP_MILLI, v3 = KI_MILLI */
    CTRL_MODE_SET_KD = 5,    /* v1 = junta 1..3, v2 = KD_MILLI */
    CTRL_MODE_SET_DUTY = 6,  /* v1 = junta 1..3, v2 = DUTY_MIN ‰, v3 = DUTY_MAX ‰ (≤ DUTY_MAX_HARD) */
} ctrl_mode_t;

/* Bits de diagnóstico que viajan en /scara/limits junto a los 6 finales (bits 0–5).
 * Extensión del contrato. */
#define STATUS_BIT_STALL    (1u << 6) /* alguna junta cortada por bloqueo */
#define STATUS_BIT_WATCHDOG (1u << 7) /* sin comandos hace > WATCHDOG_MS → STOP */

/* Inicializa motores/encoders/finales ya creados y lanza la tarea de control. */
esp_err_t control_start(void);

/* Llamar al recibir /scara/cmd. Seguro entre núcleos (sección crítica corta). */
void control_command(int32_t mode, int32_t v1, int32_t v2, int32_t v3);

/* Foto del estado para publicar: cuentas de las 3 juntas y byte de finales+diagnóstico. */
void control_get_status(int32_t counts[NUM_JOINTS], uint8_t *limits_and_flags);
