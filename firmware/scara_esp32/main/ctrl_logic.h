/*
 * ctrl_logic.h — Lógica PURA del control: PID, debounce, enclavamiento, modos, watchdog,
 * protección de bloqueo. Sin ESP-IDF, sin FreeRTOS, sin hardware, sin memoria dinámica.
 *
 * Por qué separada:
 *   - se prueba en el PC con gcc (firmware/scara_esp32/host_test, `make test`)
 *   - el simulador fake_esp32 carga ESTE MISMO código (librería compartida), así lo que
 *     se ve en simulación es el comportamiento real del firmware, no una copia en Python
 *   - control.c queda como "pegamento": lee sensores, llama a ctrl_step() y mueve motores
 *
 * Uso (cada CONTROL_PERIOD_MS):
 *   efecto = ctrl_command(&st, modo, v1, v2, v3);   // por cada mensaje /scara/cmd recibido
 *   if (efecto.zero) encoder_set(efecto.joint, efecto.value);
 *   flags = ctrl_step(&st, pos, finales, out);       // out[j] = duty ‰ a aplicar
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ctrl_config.h"

#define NJ CTRL_NUM_JOINTS

/* ---------------- Modos de /scara/cmd ---------------- */
typedef enum {
    CTRL_MODE_STOP = 0,      /* freno en las 3 juntas */
    CTRL_MODE_POSITION = 1,  /* v = metas en cuentas → PID */
    CTRL_MODE_VELOCITY = 2,  /* v = duty con signo en ‰ → lazo abierto */
    CTRL_MODE_ZERO = 3,      /* v1 = junta 1..3, v2 = valor a cargar en su contador */
    /* Extensión: configuración en caliente, no mueve nada. */
    CTRL_MODE_SET_GAINS = 4, /* v1 = junta 1..3, v2 = KP_MILLI, v3 = KI_MILLI */
    CTRL_MODE_SET_KD = 5,    /* v1 = junta 1..3, v2 = KD_MILLI */
    CTRL_MODE_SET_DUTY = 6,  /* v1 = junta 1..3, v2 = DUTY_MIN ‰, v3 = DUTY_MAX ‰ (≤ DUTY_MAX_HARD) */
} ctrl_mode_t;

/* ---------------- Bits de /scara/limits ----------------
 * 0–5: finales (bit0 FC1_MIN, bit1 FC1_MAX, ... bit5 FC3_MAX), 1 = presionado.
 * 6–7: diagnóstico (extensión §14.10). */
#define LIMIT_BIT_MIN(j) (1u << (2 * (j)))
#define LIMIT_BIT_MAX(j) (1u << (2 * (j) + 1))
#define LIMITS_MASK         0x3Fu
#define STATUS_BIT_STALL    (1u << 6) /* alguna junta cortada por bloqueo */
#define STATUS_BIT_WATCHDOG (1u << 7) /* sin comandos hace > WATCHDOG_MS → STOP */

/* ---------------- Debounce de finales ---------------- */
typedef struct {
    uint8_t stable;    /* estado aceptado */
    uint8_t candidate; /* lectura que se está "probando" */
    uint8_t count;     /* cuántas veces seguidas se ha visto */
} debounce_t;

void debounce_init(debounce_t *d, uint8_t initial);
/* Una lectura cruda (bitmask, 1 = presionado) → devuelve el estado estable. */
uint8_t debounce_update(debounce_t *d, uint8_t raw);

/* ---------------- Controlador ---------------- */
typedef struct {
    int32_t kp[NJ], ki[NJ], kd[NJ];
    int32_t duty_min[NJ], duty_max[NJ];
} ctrl_gains_t;

typedef struct {
    ctrl_gains_t g;
    int32_t mode;
    int32_t setpoint[NJ];              /* meta (cuentas) o duty (‰) según el modo */
    int64_t integ[NJ];                 /* ∫e dt en cuenta·ms */
    int32_t pos_hist[NJ][VEL_WINDOW_MS];
    int32_t hist_idx;
    bool stall_fault[NJ];
    int32_t stall_ms[NJ];
    int32_t stall_ref[NJ];
    uint32_t ms_since_cmd;
    bool watchdog_tripped;
} ctrl_state_t;

typedef struct {
    bool zero;      /* true ⇒ el llamador debe cargar 'value' en el contador de 'joint' */
    int32_t joint;  /* 0..NJ-1 */
    int32_t value;
} ctrl_effect_t;

/* Estado inicial: STOP, watchdog disparado (sin comandos no se mueve nada), ganancias
 * por defecto de ctrl_config.h, historial de velocidad lleno con 'pos'. */
void ctrl_init(ctrl_state_t *s, const int32_t pos[NJ]);

/* Procesa un mensaje de /scara/cmd. Cualquier mensaje alimenta el watchdog. */
ctrl_effect_t ctrl_command(ctrl_state_t *s, int32_t mode, int32_t v1, int32_t v2, int32_t v3);

/* Un paso del lazo (CONTROL_PERIOD_MS). pos = cuentas actuales, limits = finales con
 * debounce (bits 0–5). Escribe out[j] (‰, 0 = freno) y devuelve el byte de estado
 * (finales + bits de diagnóstico) que se publica en /scara/limits. */
uint8_t ctrl_step(ctrl_state_t *s, const int32_t pos[NJ], uint8_t limits, int32_t out[NJ]);

/* Para el simulador (ctypes): tamaño del estado, para reservarlo desde Python. */
size_t ctrl_state_size(void);
size_t debounce_state_size(void);
