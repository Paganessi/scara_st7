/*
 * limit_switches.h — (se llama así y no limits.h para no tapar el <limits.h> del sistema)
 * Finales de carrera con debounce.
 *
 * Bitmask (igual que /scara/limits): bit0 FC1_MIN · bit1 FC1_MAX · bit2 FC2_MIN ·
 * bit3 FC2_MAX · bit4 FC3_MIN · bit5 FC3_MAX. 1 = presionado.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define LIMIT_BIT_MIN(j) (1u << (2 * (j)))
#define LIMIT_BIT_MAX(j) (1u << (2 * (j) + 1))

esp_err_t limits_init(void);

/* Llamar cada 1 ms desde la tarea de control: lee los pines y aplica el debounce. */
void limits_update(void);

/* Último estado estable (con debounce). */
uint8_t limits_get(void);

/* Lectura cruda sin debounce (solo para diagnóstico en la Fase 1). */
uint8_t limits_read_raw(void);
