/*
 * motor.h — Manejo de los 3 motores DC a través de 2× TB6612FNG.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* Configura LEDC (PWM 20 kHz, 10 bits) y los pines IN1/IN2. Deja todo en freno. */
esp_err_t motor_init(void);

/* duty_permille con signo (−1000…1000). El signo es la dirección (después de aplicar
 * MOTOR_INVERT). Se satura a ±1000; la saturación de seguridad (DUTY_MAX) la hace control.c.
 * 0 ⇒ freno. */
void motor_set(int joint, int32_t duty_permille);

/* Freno activo (IN1=IN2=1, cortocircuita el bobinado: para rápido y sujeta). */
void motor_brake(int joint);
void motor_brake_all(void);

/* Invierte (o no) el sentido del motor en caliente. Útil en la prueba de Fase 1. */
void motor_set_invert(int joint, bool invert);
