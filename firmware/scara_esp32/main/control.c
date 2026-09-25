/*
 * control.c — Lazo de control a 1 kHz en el core 1.
 *
 * Por qué aquí y no en ROS: el viaje ida y vuelta por USB+DDS tarda
 * varios ms y con jitter; un PID con retardo variable se desestabiliza. Además el
 * enclavamiento con finales y el watchdog tienen que reaccionar en ~1 ms.
 *
 * Cada milisegundo:
 *   1. lee finales (debounce) y encoders
 *   2. toma el último comando (si llegó uno nuevo)
 *   3. watchdog: sin comandos en WATCHDOG_MS ⇒ STOP
 *   4. calcula la salida según el modo (STOP / POSITION con PID / VELOCITY)
 *   5. enclavamiento con finales + protección de bloqueo
 *   6. aplica a los motores y deja una foto del estado para micro-ROS
 *
 * Todo en enteros (sin floats) y sin memoria dinámica.
 */
#include "control.h"

#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "config.h"
#include "encoder.h"
#include "limit_switches.h"
#include "motor.h"

static const char *TAG = "control";

/* ---------- Parámetros por junta ----------
 * Arrancan con los valores de config.h; ROS puede cambiarlos en caliente con los
 * modos SET_GAINS / SET_DUTY (así se sintoniza sin reflashear y los valores finales
 * viven en el YAML, como el resto de la calibración). */
typedef struct {
    int32_t kp[NUM_JOINTS], ki[NUM_JOINTS], kd[NUM_JOINTS];
    int32_t duty_min[NUM_JOINTS], duty_max[NUM_JOINTS];
} gains_t;
static gains_t s_g = {
    .kp = KP_MILLI_INIT, .ki = KI_MILLI_INIT, .kd = KD_MILLI_INIT,
    .duty_min = DUTY_MIN_INIT, .duty_max = DUTY_MAX_INIT,
};                      /* copia privada de la tarea de control */
static gains_t s_g_shared; /* la que escribe control_command (bajo s_mux) */

/* ---------- Estado compartido entre núcleos (protegido por s_mux) ---------- */
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static struct {
    uint32_t alive;         /* +1 con CUALQUIER mensaje (alimenta el watchdog) */
    uint32_t gains_seq;     /* +1 cuando cambian ganancias/duty */
    uint32_t seq;           /* +1 con cada comando de movimiento (STOP/POSITION/VELOCITY) */
    int32_t mode;
    int32_t v[NUM_JOINTS];
    bool zero_pending;      /* ZERO va aparte para que un comando siguiente no lo pise */
    int32_t zero_joint;     /* 0..2 */
    int32_t zero_value;
} s_cmd;
static struct {
    int32_t counts[NUM_JOINTS];
    uint8_t flags;
} s_status;

/* ---------- Estado privado de la tarea de control ---------- */
static int32_t s_mode = CTRL_MODE_STOP;
static int32_t s_setpoint[NUM_JOINTS];      /* meta (cuentas) o duty (‰) según el modo */
static int64_t s_integ[NUM_JOINTS];         /* ∫e dt en cuenta·ms */
static int32_t s_pos_hist[NUM_JOINTS][VEL_WINDOW_MS];
static int s_hist_idx;
static bool s_stall_fault[NUM_JOINTS];
static int32_t s_stall_ms[NUM_JOINTS];
static int32_t s_stall_ref[NUM_JOINTS];

static inline int32_t clamp_i32(int32_t x, int32_t lo, int32_t hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static void reset_controllers(void)
{
    memset(s_integ, 0, sizeof(s_integ));
    memset(s_stall_fault, 0, sizeof(s_stall_fault));
    memset(s_stall_ms, 0, sizeof(s_stall_ms));
}

/* Rellena el historial de posiciones para que la derivada no pegue un salto. */
static void reset_velocity_history(int j, int32_t pos)
{
    for (int k = 0; k < VEL_WINDOW_MS; k++) s_pos_hist[j][k] = pos;
}

/* PID de posición de una junta. Devuelve duty en ‰ ya saturado a ±DUTY_MAX. */
static int32_t pid_step(int j, int32_t pos, int32_t vel_cps)
{
    const int32_t e = s_setpoint[j] - pos;
    if (e >= -POS_TOLERANCE_COUNTS && e <= POS_TOLERANCE_COUNTS) {
        /* Llegó: soltar. Si no, la compensación de zona muerta lo haría temblar. */
        return 0;
    }

    /* Integrador con clamp (anti-windup): |KI·∫e| nunca pasa de I_TERM_MAX_PERMILLE. */
    int64_t integ_prev = s_integ[j];
    int64_t i_term = 0;
    if (s_g.ki[j] > 0) {
        const int64_t integ_max = ((int64_t)I_TERM_MAX_PERMILLE * 1000000) / s_g.ki[j];
        s_integ[j] += (int64_t)e * CONTROL_PERIOD_MS;
        if (s_integ[j] > integ_max) s_integ[j] = integ_max;
        if (s_integ[j] < -integ_max) s_integ[j] = -integ_max;
        i_term = ((int64_t)s_g.ki[j] * s_integ[j]) / 1000000; /* (‰/(c·s))·(c·ms) */
    }

    const int64_t p_term = ((int64_t)s_g.kp[j] * e) / 1000;
    /* Derivada sobre la MEDICIÓN: un salto de la meta no produce un "patadón". */
    const int64_t d_term = ((int64_t)s_g.kd[j] * vel_cps) / 1000;

    int64_t u = p_term + i_term - d_term;
    /* Compensación de zona muerta: por debajo de DUTY_MIN el motor no arranca. */
    if (u > 0) u += s_g.duty_min[j];
    if (u < 0) u -= s_g.duty_min[j];

    const int32_t umax = s_g.duty_max[j];
    if (u > umax || u < -umax) {
        /* Saturado empujando en el mismo sentido del error: no seguir integrando. */
        if ((u > 0) == (e > 0)) s_integ[j] = integ_prev;
        u = u > 0 ? umax : -umax;
    }
    return (int32_t)u;
}

/* Detección de bloqueo: mucho duty y el encoder casi no se mueve. */
static void stall_check(int j, int32_t out, int32_t pos)
{
    const int32_t mag = out >= 0 ? out : -out;
    if (mag < STALL_DUTY_PERMILLE) {
        s_stall_ms[j] = 0;
        return;
    }
    if (s_stall_ms[j] == 0) s_stall_ref[j] = pos;
    s_stall_ms[j] += CONTROL_PERIOD_MS;
    if (s_stall_ms[j] >= STALL_TIME_MS) {
        const int32_t moved = pos - s_stall_ref[j];
        if (moved < STALL_MIN_COUNTS && moved > -STALL_MIN_COUNTS) {
            s_stall_fault[j] = true;
            ESP_LOGW(TAG, "bloqueo en junta %d: corte", j + 1);
        }
        s_stall_ms[j] = 0; /* nueva ventana */
    }
}

static void control_task(void *arg)
{
    (void)arg;
    uint32_t last_seq = 0, last_alive = 0, last_gains_seq = 0;
    TickType_t last_cmd_tick = xTaskGetTickCount();
    TickType_t wake = xTaskGetTickCount();
    bool watchdog_tripped = true; /* al arrancar no hay comandos: STOP */

    for (int j = 0; j < NUM_JOINTS; j++) reset_velocity_history(j, encoder_get(j));

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
        const TickType_t now = xTaskGetTickCount();

        /* 1. Sensores */
        limits_update();
        const uint8_t lim = limits_get();

        /* 2. Comando nuevo (copia bajo sección crítica, procesado afuera) */
        uint32_t seq, alive, gseq;
        int32_t mode, v[NUM_JOINTS];
        bool zero;
        int32_t zj = 0, zv = 0;
        portENTER_CRITICAL(&s_mux);
        alive = s_cmd.alive;
        gseq = s_cmd.gains_seq;
        if (gseq != last_gains_seq) s_g = s_g_shared;
        seq = s_cmd.seq;
        mode = s_cmd.mode;
        memcpy(v, s_cmd.v, sizeof(v));
        zero = s_cmd.zero_pending;
        if (zero) {
            zj = s_cmd.zero_joint;
            zv = s_cmd.zero_value;
            s_cmd.zero_pending = false;
        }
        portEXIT_CRITICAL(&s_mux);

        if (zero) {
            /* ZERO: cargar el contador y quedar en STOP (cambiar la referencia con metas
             * de POSITION viejas produciría un salto). */
            encoder_set(zj, zv);
            reset_velocity_history(zj, zv);
            s_mode = CTRL_MODE_STOP;
            reset_controllers();
        }

        int32_t pos[NUM_JOINTS];
        for (int j = 0; j < NUM_JOINTS; j++) pos[j] = encoder_get(j);

        if (gseq != last_gains_seq) {
            last_gains_seq = gseq;
            memset(s_integ, 0, sizeof(s_integ)); /* ganancias nuevas: integrador desde cero */
        }
        if (alive != last_alive) {
            /* Cualquier mensaje en /scara/cmd alimenta el watchdog. */
            last_alive = alive;
            last_cmd_tick = now;
            watchdog_tripped = false;
        }
        if (seq != last_seq) {
            last_seq = seq;
            if (mode != s_mode || mode == CTRL_MODE_STOP) {
                /* Cambio de modo o STOP explícito: limpiar integradores y fallas. */
                reset_controllers();
            }
            s_mode = mode;
            memcpy(s_setpoint, v, sizeof(v));
        }

        /* 3. Watchdog de comandos */
        if ((now - last_cmd_tick) > pdMS_TO_TICKS(WATCHDOG_MS)) {
            if (!watchdog_tripped) ESP_LOGW(TAG, "watchdog: sin /scara/cmd → STOP");
            watchdog_tripped = true;
            s_mode = CTRL_MODE_STOP;
        }

        /* Velocidad por diferencia de posiciones en una ventana de VEL_WINDOW_MS */
        int32_t vel_cps[NUM_JOINTS];
        for (int j = 0; j < NUM_JOINTS; j++) {
            vel_cps[j] = (pos[j] - s_pos_hist[j][s_hist_idx]) * (1000 / VEL_WINDOW_MS);
            s_pos_hist[j][s_hist_idx] = pos[j];
        }
        s_hist_idx = (s_hist_idx + 1) % VEL_WINDOW_MS;

        /* 4–5. Salida por junta */
        uint8_t flags = lim;
        for (int j = 0; j < NUM_JOINTS; j++) {
            int32_t out = 0;
            switch (s_mode) {
            case CTRL_MODE_POSITION:
                out = pid_step(j, pos[j], vel_cps[j]);
                break;
            case CTRL_MODE_VELOCITY:
                out = clamp_i32(s_setpoint[j], -s_g.duty_max[j], s_g.duty_max[j]);
                break;
            default:
                out = 0;
                break;
            }

            /* Enclavamiento (SIEMPRE): no empujar contra un final presionado.
             * Convención: duty>0 va hacia MAX, duty<0 hacia MIN. */
            if (((lim & LIMIT_BIT_MIN(j)) && out < 0) || ((lim & LIMIT_BIT_MAX(j)) && out > 0)) {
                out = 0;
                s_integ[j] = 0;
            }

            if (s_stall_fault[j]) out = 0;
            stall_check(j, out, pos[j]);
            if (s_stall_fault[j]) {
                out = 0;
                flags |= STATUS_BIT_STALL;
            }

            /* 6. Aplicar */
            if (out == 0) motor_brake(j);
            else motor_set(j, out);
        }
        if (watchdog_tripped) flags |= STATUS_BIT_WATCHDOG;

        portENTER_CRITICAL(&s_mux);
        memcpy(s_status.counts, pos, sizeof(pos));
        s_status.flags = flags;
        portEXIT_CRITICAL(&s_mux);
    }
}

esp_err_t control_start(void)
{
    memset(&s_cmd, 0, sizeof(s_cmd));
    s_cmd.mode = CTRL_MODE_STOP;
    s_g_shared = s_g;
    motor_brake_all();
    BaseType_t ok = xTaskCreatePinnedToCore(control_task, "control", CONTROL_TASK_STACK, NULL,
                                            CONTROL_TASK_PRIO, NULL, CONTROL_TASK_CORE);
    ESP_LOGI(TAG, "tarea de control %s (core %d, %d ms)", ok == pdPASS ? "lanzada" : "FALLÓ",
             CONTROL_TASK_CORE, CONTROL_PERIOD_MS);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}

void control_command(int32_t mode, int32_t v1, int32_t v2, int32_t v3)
{
    /* Modos desconocidos se tratan como STOP (ante la duda, frenar). */
    if (mode < CTRL_MODE_STOP || mode > CTRL_MODE_SET_DUTY) mode = CTRL_MODE_STOP;
    /* v1 de ZERO/SET_* viene 1..3 en el protocolo; internamente 0..2. */
    const int j = v1 - 1;
    const bool j_ok = (j >= 0 && j < NUM_JOINTS);

    portENTER_CRITICAL(&s_mux);
    s_cmd.alive++;
    switch (mode) {
    case CTRL_MODE_ZERO:
        /* Índice inválido ⇒ se ignora el ZERO (igual cuenta como keepalive). */
        if (j_ok) {
            s_cmd.zero_pending = true;
            s_cmd.zero_joint = j;
            s_cmd.zero_value = v2;
        }
        break;
    case CTRL_MODE_SET_GAINS:
        if (j_ok) {
            s_g_shared.kp[j] = clamp_i32(v2, 0, 1000000);
            s_g_shared.ki[j] = clamp_i32(v3, 0, 1000000);
            s_cmd.gains_seq++;
        }
        break;
    case CTRL_MODE_SET_KD:
        if (j_ok) {
            s_g_shared.kd[j] = clamp_i32(v2, 0, 1000000);
            s_cmd.gains_seq++;
        }
        break;
    case CTRL_MODE_SET_DUTY:
        if (j_ok) {
            /* Tope duro: ni desde ROS se puede pasar de DUTY_MAX_HARD (protege al TB6612). */
            s_g_shared.duty_min[j] = clamp_i32(v2, 0, DUTY_MAX_HARD / 2);
            s_g_shared.duty_max[j] = clamp_i32(v3, 0, DUTY_MAX_HARD);
            s_cmd.gains_seq++;
        }
        break;
    default: /* STOP / POSITION / VELOCITY */
        s_cmd.seq++;
        s_cmd.mode = mode;
        s_cmd.v[0] = v1;
        s_cmd.v[1] = v2;
        s_cmd.v[2] = v3;
        break;
    }
    portEXIT_CRITICAL(&s_mux);
}

void control_get_status(int32_t counts[NUM_JOINTS], uint8_t *limits_and_flags)
{
    portENTER_CRITICAL(&s_mux);
    memcpy(counts, s_status.counts, sizeof(s_status.counts));
    *limits_and_flags = s_status.flags;
    portEXIT_CRITICAL(&s_mux);
}
