/*
 * config.h — Constantes del firmware (todo lo que no es un número de pin).
 *
 * Aquí SOLO va lo que el micro necesita para tiempo real. Todo lo "calibrable
 * en unidades físicas" (relaciones de transmisión, límites en rad/m, homing)
 * vive en ROS: ros2_ws/src/scara_bringup/config/scara.yaml.
 *
 * Convención de signos, la que usa el enclavamiento:
 *     duty positivo ⇒ las cuentas suben ⇒ la junta se aleja de MIN y va hacia MAX
 * Se ajusta en la Fase 1 con MOTOR_INVERT y ENCODER_INVERT.
 *
 * Unidades: duty en ‰ (−1000…1000), posiciones en cuentas de encoder (x4),
 * tiempos en ms. Sin floats: las ganancias van en "mili-unidades" (×1000).
 */
#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h" /* configMAX_PRIORITIES */
#include "pins.h"

/* ---------------- PWM (LEDC) ----------------
 * 20 kHz: por encima de lo audible (no chilla el motor) y dentro de lo que
 * el TB6612 acepta (máx. 100 kHz). 10 bits → 1024 niveles; 20 kHz·1024 ≈ 20 MHz < 80 MHz APB. */
#define PWM_FREQ_HZ        20000
#define PWM_RES_BITS       10
#define DUTY_FULL_PERMILLE 1000

/* ---------------- Periodos ---------------- */
#define CONTROL_PERIOD_MS  1    /* lazo de control a 1 kHz (requiere CONFIG_FREERTOS_HZ=1000) */
#define PUBLISH_PERIOD_MS  20   /* /scara/enc_counts y /scara/limits a 50 Hz */
#define WATCHDOG_MS        500  /* sin /scara/cmd durante esto ⇒ STOP */

/* ---------------- Finales de carrera ----------------
 * Debounce: el estado solo cambia tras N lecturas iguales seguidas (1 lectura/ms). */
#define LIMIT_DEBOUNCE_N   5

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

/* ---------------- Saturaciones de potencia ----------------
 * La corriente de bloqueo del Pololu 37D (~5 A a 12 V) supera al TB6612 (3.2 A pico),
 * así que nunca damos 100 %. DUTY_MAX aplica en TODOS los modos. */
#define DUTY_MAX_INIT       {600, 600, 600}
/* Tope absoluto: ni desde ROS (modo SET_DUTY) se puede pedir más que esto. */
#define DUTY_MAX_HARD       800

/* Compensación de zona muerta: duty mínimo que vence la fricción estática.
 * Se calibra por junta en la Fase 3 (subir duty poco a poco hasta que arranque). */
#define DUTY_MIN_INIT       {0, 0, 0}  /* TODO calibrar en Fase 3 */

/* ---------------- PID de posición (en cuentas) ----------------
 * u[‰] = KP·e + KI·∫e dt − KD·dpos/dt   (derivada sobre la MEDICIÓN)
 *   KP_MILLI: ‰ por cuenta          ×1000   (1000 → 1 ‰ por cuenta de error)
 *   KI_MILLI: ‰ por (cuenta·s)      ×1000
 *   KD_MILLI: ‰ por (cuenta/s)      ×1000
 * Arranque de la sintonía: solo P, bajo (Fase 3). */
#define KP_MILLI_INIT       {1000, 1000, 1000}
#define KI_MILLI_INIT       {0, 0, 0}
#define KD_MILLI_INIT       {0, 0, 0}
/* Límite del término integral (anti-windup por clamp), en ‰. */
#define I_TERM_MAX_PERMILLE 200
/* Banda muerta de posición: con |error| ≤ esto se considera "llegó" y se suelta el motor
 * (evita que la compensación de zona muerta lo haga temblar alrededor de la meta). */
#define POS_TOLERANCE_COUNTS 4
/* Ventana para estimar velocidad (derivada) por diferencia de posiciones. */
#define VEL_WINDOW_MS       10

/* ---------------- Protección por bloqueo (stall) ----------------
 * Si pedimos más de STALL_DUTY ‰ durante STALL_TIME_MS y el encoder se movió menos de
 * STALL_MIN_COUNTS, el motor está trabado (tope mecánico, cable, etc.): se corta esa junta
 * hasta que llegue un STOP o cambie el modo. Protege al TB6612 y al motor. */
#define STALL_DUTY_PERMILLE 350
#define STALL_TIME_MS       300
#define STALL_MIN_COUNTS    8

/* ---------------- Tareas FreeRTOS ---------------- */
#define CONTROL_TASK_CORE   1
#define CONTROL_TASK_PRIO   (configMAX_PRIORITIES - 2)
#define CONTROL_TASK_STACK  4096
#define UROS_TASK_CORE      0
#define UROS_TASK_PRIO      5
#define UROS_TASK_STACK     16000

/* ---------------- micro-ROS ---------------- */
#define UROS_BAUDRATE       115200  /* igual al -b del micro_ros_agent */
