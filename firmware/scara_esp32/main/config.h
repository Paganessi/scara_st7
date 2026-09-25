/*
 * config.h — Constantes del firmware (todo lo que no es un número de pin).
 *
 * Las constantes de la LÓGICA de control (PID, watchdog, debounce, duty, stall) están en
 * ctrl_config.h (se incluye aquí) para poder compilarlas también en el PC.
 * Aquí queda lo que depende del hardware/ESP-IDF.
 *
 * Todo lo "calibrable en unidades físicas" (relaciones de transmisión, límites en rad/m,
 * homing) vive en ROS: ros2_ws/src/scara_bringup/config/scara.yaml.
 *
 * Convención de signos, la que usa el enclavamiento:
 *     duty positivo ⇒ las cuentas suben ⇒ la junta se aleja de MIN y va hacia MAX
 * Se ajusta en la Fase 1 con MOTOR_INVERT y ENCODER_INVERT.
 */
#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h" /* configMAX_PRIORITIES */
#include "ctrl_config.h"
#include "pins.h"

/* ---------------- PWM (LEDC) ----------------
 * 20 kHz: por encima de lo audible (no chilla el motor) y dentro de lo que
 * el TB6612 acepta (máx. 100 kHz). 10 bits → 1024 niveles; 20 kHz·1024 ≈ 20 MHz < 80 MHz APB. */
#define PWM_FREQ_HZ        20000
#define PWM_RES_BITS       10

/* ---------------- Publicación ---------------- */
#define PUBLISH_PERIOD_MS  20   /* /scara/enc_counts y /scara/limits a 50 Hz */

/* ---------------- Encoders (PCNT) ----------------
 * Filtro de glitch: ignora pulsos más cortos que esto. A 12 V y ~10 000 rpm de motor
 * el encoder da ~10.7 k cuentas/s → un flanco cada ~93 µs, así que 1 µs no se come
 * pulsos reales pero sí el ruido de conmutación del PWM y la errata de GPIO36/39. */
#define ENC_GLITCH_NS      1000
/* Límites del contador hardware (16 bits con signo). Al llegar ahí el driver acumula
 * en software (accum_count) y seguimos contando en 32 bits. */
#define ENC_PCNT_HIGH      30000
#define ENC_PCNT_LOW       (-30000)

/* ---------------- Signos por junta (AJUSTAR EN FASE 1) ----------------
 * MOTOR_INVERT[j]=1   → invierte el sentido del motor j (como cruzar sus cables).
 * ENCODER_INVERT[j]=1 → invierte el sentido de conteo (como cruzar A y B).
 * Orden: {θ1, θ2, Z}.  Valores por defecto = sin invertir, pendientes de la Fase 1. */
#define MOTOR_INVERT_INIT   {0, 0, 0}  /* TODO CONFIRMAR en Fase 1 */
#define ENCODER_INVERT_INIT {0, 0, 0}  /* TODO CONFIRMAR en Fase 1 */

/* ---------------- Tareas FreeRTOS ---------------- */
#define CONTROL_TASK_CORE   1
#define CONTROL_TASK_PRIO   (configMAX_PRIORITIES - 2)
#define CONTROL_TASK_STACK  4096
#define CMD_QUEUE_LEN       16  /* mensajes /scara/cmd en cola entre núcleos */
#define UROS_TASK_CORE      0
#define UROS_TASK_PRIO      5
#define UROS_TASK_STACK     16000

/* ---------------- micro-ROS ---------------- */
#define UROS_BAUDRATE       115200  /* igual al -b del micro_ros_agent */
