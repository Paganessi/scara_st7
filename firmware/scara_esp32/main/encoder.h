/*
 * encoder.h — Lectura de los 3 encoders de cuadratura con el periférico PCNT.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* Configura 3 unidades PCNT en modo x4, con filtro de glitch y acumulación a 32 bits. */
esp_err_t encoder_init(void);

/* Posición acumulada en cuentas (ya con ENCODER_INVERT aplicado). */
int32_t encoder_get(int joint);

/* Fuerza el contador a 'value' (lo usa el homing vía modo ZERO). */
void encoder_set(int joint, int32_t value);

/* Invierte (o no) el sentido de conteo en caliente. Útil en la prueba de Fase 1.
 * Conserva la posición actual (no pega saltos). */
void encoder_set_invert(int joint, bool invert);
