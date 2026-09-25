/*
 * ctrl_config.h — Constantes de la LÓGICA de control (sin nada de hardware ni ESP-IDF).
 *
 * Separado de config.h para que ctrl_logic.c compile también en el PC (pruebas
 * unitarias con gcc y simulador fake_esp32). config.h lo incluye, así que en el
 * firmware todo sigue en un solo lugar a efectos prácticos.
 *
 * Unidades: duty en ‰ (−1000…1000), posiciones en cuentas (x4), tiempos en ms.
 * Sin floats: las ganancias van en "mili-unidades" (×1000).
 */
#pragma once

#define CTRL_NUM_JOINTS    3

/* ---------------- Periodos ---------------- */
#define CONTROL_PERIOD_MS  1    /* lazo de control a 1 kHz */
#define WATCHDOG_MS        500  /* sin /scara/cmd durante esto ⇒ STOP */

/* ---------------- Finales de carrera ----------------
 * Debounce: el estado solo cambia tras N lecturas iguales seguidas (1 lectura/ms). */
#define LIMIT_DEBOUNCE_N   5

/* ---------------- Saturaciones de potencia ----------------
 * La corriente de bloqueo del Pololu 37D (~5 A a 12 V) supera al TB6612 (3.2 A pico),
 * así que nunca damos 100 %. DUTY_MAX aplica en TODOS los modos. */
#define DUTY_FULL_PERMILLE 1000
#define DUTY_MAX_INIT       {600, 600, 600}
/* Tope absoluto: ni desde ROS (modo SET_DUTY) se puede pedir más que esto. */
#define DUTY_MAX_HARD       800

/* Compensación de zona muerta: duty mínimo que vence la fricción estática.
 * Se calibra por junta en la Fase 3 (deadzone_test). */
#define DUTY_MIN_INIT       {0, 0, 0}  /* TODO calibrar en Fase 3 */

/* ---------------- PID de posición (en cuentas) ----------------
 * u[‰] = KP·e + KI·∫e dt − KD·dpos/dt   (derivada sobre la MEDICIÓN)
 *   KP_MILLI: ‰ por cuenta          ×1000   (1000 → 1 ‰ por cuenta de error)
 *   KI_MILLI: ‰ por (cuenta·s)      ×1000
 *   KD_MILLI: ‰ por (cuenta/s)      ×1000
 * Valores de arranque; ROS los reemplaza en caliente (modos 4–6) desde scara.yaml. */
#define KP_MILLI_INIT       {1000, 1000, 1000}
#define KI_MILLI_INIT       {0, 0, 0}
#define KD_MILLI_INIT       {0, 0, 0}
/* Límite del término integral (anti-windup por clamp), en ‰. */
#define I_TERM_MAX_PERMILLE 200
/* Banda muerta de posición: con |error| ≤ esto se considera "llegó" y se suelta el motor. */
#define POS_TOLERANCE_COUNTS 4
/* Ventana para estimar velocidad (derivada) por diferencia de posiciones. */
#define VEL_WINDOW_MS       10

/* ---------------- Protección por bloqueo (stall) ----------------
 * Si pedimos más de STALL_DUTY ‰ durante STALL_TIME_MS y el encoder se movió menos de
 * STALL_MIN_COUNTS, el motor está trabado: se corta esa junta hasta un STOP o un cambio
 * de modo. Protege al TB6612 y al motor. */
#define STALL_DUTY_PERMILLE 350
#define STALL_TIME_MS       300
#define STALL_MIN_COUNTS    8
