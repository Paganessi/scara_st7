/*
 * ctrl_logic.c — Lógica pura del control (ver ctrl_logic.h). Enteros, sin malloc.
 *
 * Contenido:
 *   - debounce de finales por conteo
 *   - máquina de modos (STOP / POSITION / VELOCITY / ZERO / configuración)
 *   - PID de posición en cuentas: derivada sobre la medición, anti-windup por clamp +
 *     integración condicional, compensación de zona muerta, banda de llegada
 *   - enclavamiento con finales (siempre activo, en cualquier modo)
 *   - protección de bloqueo (stall) y watchdog de comandos
 */
#include "ctrl_logic.h"

#include <string.h>

static const int32_t k_kp[NJ] = KP_MILLI_INIT;
static const int32_t k_ki[NJ] = KI_MILLI_INIT;
static const int32_t k_kd[NJ] = KD_MILLI_INIT;
static const int32_t k_dmin[NJ] = DUTY_MIN_INIT;
static const int32_t k_dmax[NJ] = DUTY_MAX_INIT;

static inline int32_t clamp_i32(int32_t x, int32_t lo, int32_t hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

/* ======================= Debounce ======================= */

void debounce_init(debounce_t *d, uint8_t initial)
{
    d->stable = initial;
    d->candidate = initial;
    d->count = LIMIT_DEBOUNCE_N;
}

uint8_t debounce_update(debounce_t *d, uint8_t raw)
{
    if (raw == d->candidate) {
        if (d->count < LIMIT_DEBOUNCE_N) d->count++;
    } else {
        d->candidate = raw;
        d->count = 1;
    }
    if (d->count >= LIMIT_DEBOUNCE_N) d->stable = d->candidate;
    return d->stable;
}

/* ======================= Controlador ======================= */

static void reset_controllers(ctrl_state_t *s)
{
    memset(s->integ, 0, sizeof(s->integ));
    memset(s->stall_fault, 0, sizeof(s->stall_fault));
    memset(s->stall_ms, 0, sizeof(s->stall_ms));
}

static void reset_history(ctrl_state_t *s, int j, int32_t pos)
{
    for (int k = 0; k < VEL_WINDOW_MS; k++) s->pos_hist[j][k] = pos;
}

void ctrl_init(ctrl_state_t *s, const int32_t pos[NJ])
{
    memset(s, 0, sizeof(*s));
    memcpy(s->g.kp, k_kp, sizeof(k_kp));
    memcpy(s->g.ki, k_ki, sizeof(k_ki));
    memcpy(s->g.kd, k_kd, sizeof(k_kd));
    memcpy(s->g.duty_min, k_dmin, sizeof(k_dmin));
    memcpy(s->g.duty_max, k_dmax, sizeof(k_dmax));
    s->mode = CTRL_MODE_STOP;
    s->watchdog_tripped = true;          /* al arrancar no hay comandos: STOP */
    s->ms_since_cmd = WATCHDOG_MS + 1;
    for (int j = 0; j < NJ; j++) reset_history(s, j, pos[j]);
}

ctrl_effect_t ctrl_command(ctrl_state_t *s, int32_t mode, int32_t v1, int32_t v2, int32_t v3)
{
    ctrl_effect_t eff = {.zero = false, .joint = 0, .value = 0};
    /* Cualquier mensaje en /scara/cmd alimenta el watchdog. */
    s->ms_since_cmd = 0;
    s->watchdog_tripped = false;

    /* Modos desconocidos se tratan como STOP (ante la duda, frenar). */
    if (mode < CTRL_MODE_STOP || mode > CTRL_MODE_SET_DUTY) mode = CTRL_MODE_STOP;
    /* v1 de ZERO/SET_* viene 1..3 en el protocolo; internamente 0..2. */
    const int j = v1 - 1;
    const bool j_ok = (j >= 0 && j < NJ);

    switch (mode) {
    case CTRL_MODE_ZERO:
        /* Índice inválido ⇒ se ignora (igual contó como keepalive). ZERO deja todo en STOP:
         * cambiar la referencia con metas de POSITION viejas produciría un salto. */
        if (j_ok) {
            eff.zero = true;
            eff.joint = j;
            eff.value = v2;
            reset_history(s, j, v2);
            s->mode = CTRL_MODE_STOP;
            reset_controllers(s);
        }
        break;
    case CTRL_MODE_SET_GAINS:
        if (j_ok) {
            s->g.kp[j] = clamp_i32(v2, 0, 1000000);
            s->g.ki[j] = clamp_i32(v3, 0, 1000000);
            s->integ[j] = 0;
        }
        break;
    case CTRL_MODE_SET_KD:
        if (j_ok) s->g.kd[j] = clamp_i32(v2, 0, 1000000);
        break;
    case CTRL_MODE_SET_DUTY:
        if (j_ok) {
            /* Tope duro: ni desde ROS se puede pasar de DUTY_MAX_HARD (protege al TB6612).
             * duty_max = 0 deja la junta siempre frenada (así se "deshabilita" una junta). */
            s->g.duty_min[j] = clamp_i32(v2, 0, DUTY_MAX_HARD / 2);
            s->g.duty_max[j] = clamp_i32(v3, 0, DUTY_MAX_HARD);
            s->integ[j] = 0;
        }
        break;
    default: /* STOP / POSITION / VELOCITY */
        if (mode != s->mode || mode == CTRL_MODE_STOP) {
            /* Cambio de modo o STOP explícito: limpiar integradores y fallas. */
            reset_controllers(s);
        }
        s->mode = mode;
        s->setpoint[0] = v1;
        s->setpoint[1] = v2;
        s->setpoint[2] = v3;
        break;
    }
    return eff;
}

/* PID de posición de una junta. Devuelve duty en ‰ ya saturado a ±duty_max. */
static int32_t pid_step(ctrl_state_t *s, int j, int32_t pos, int32_t vel_cps)
{
    const int32_t e = s->setpoint[j] - pos;
    if (e >= -POS_TOLERANCE_COUNTS && e <= POS_TOLERANCE_COUNTS) {
        /* Llegó: soltar. Si no, la compensación de zona muerta lo haría temblar. */
        return 0;
    }

    /* Integrador con clamp (anti-windup): |KI·∫e| nunca pasa de I_TERM_MAX_PERMILLE. */
    const int64_t integ_prev = s->integ[j];
    int64_t i_term = 0;
    if (s->g.ki[j] > 0) {
        const int64_t integ_max = ((int64_t)I_TERM_MAX_PERMILLE * 1000000) / s->g.ki[j];
        s->integ[j] += (int64_t)e * CONTROL_PERIOD_MS;
        if (s->integ[j] > integ_max) s->integ[j] = integ_max;
        if (s->integ[j] < -integ_max) s->integ[j] = -integ_max;
        i_term = ((int64_t)s->g.ki[j] * s->integ[j]) / 1000000; /* (‰/(c·s))·(c·ms) */
    }

    const int64_t p_term = ((int64_t)s->g.kp[j] * e) / 1000;
    /* Derivada sobre la MEDICIÓN: un salto de la meta no produce un "patadón". */
    const int64_t d_term = ((int64_t)s->g.kd[j] * vel_cps) / 1000;

    int64_t u = p_term + i_term - d_term;
    /* Compensación de zona muerta: por debajo de duty_min el motor no arranca. */
    if (u > 0) u += s->g.duty_min[j];
    if (u < 0) u -= s->g.duty_min[j];

    const int32_t umax = s->g.duty_max[j];
    if (u > umax || u < -umax) {
        /* Saturado empujando en el mismo sentido del error: no seguir integrando. */
        if ((u > 0) == (e > 0)) s->integ[j] = integ_prev;
        u = u > 0 ? umax : -umax;
    }
    return (int32_t)u;
}

/* Detección de bloqueo: mucho duty y el encoder casi no se mueve. */
static void stall_check(ctrl_state_t *s, int j, int32_t out, int32_t pos)
{
    const int32_t mag = out >= 0 ? out : -out;
    if (mag < STALL_DUTY_PERMILLE) {
        s->stall_ms[j] = 0;
        return;
    }
    if (s->stall_ms[j] == 0) s->stall_ref[j] = pos;
    s->stall_ms[j] += CONTROL_PERIOD_MS;
    if (s->stall_ms[j] >= STALL_TIME_MS) {
        const int32_t moved = pos - s->stall_ref[j];
        if (moved < STALL_MIN_COUNTS && moved > -STALL_MIN_COUNTS) s->stall_fault[j] = true;
        s->stall_ms[j] = 0; /* nueva ventana */
    }
}

uint8_t ctrl_step(ctrl_state_t *s, const int32_t pos[NJ], uint8_t limits, int32_t out[NJ])
{
    /* Watchdog de comandos */
    if (s->ms_since_cmd < 0x7FFFFFFFu) s->ms_since_cmd += CONTROL_PERIOD_MS;
    if (s->ms_since_cmd > WATCHDOG_MS) {
        s->watchdog_tripped = true;
        s->mode = CTRL_MODE_STOP;
    }

    /* Velocidad por diferencia de posiciones en una ventana de VEL_WINDOW_MS */
    int32_t vel_cps[NJ];
    for (int j = 0; j < NJ; j++) {
        vel_cps[j] = (pos[j] - s->pos_hist[j][s->hist_idx]) * (1000 / VEL_WINDOW_MS);
        s->pos_hist[j][s->hist_idx] = pos[j];
    }
    s->hist_idx = (s->hist_idx + 1) % VEL_WINDOW_MS;

    uint8_t flags = (uint8_t)(limits & LIMITS_MASK);
    for (int j = 0; j < NJ; j++) {
        int32_t u = 0;
        switch (s->mode) {
        case CTRL_MODE_POSITION:
            u = pid_step(s, j, pos[j], vel_cps[j]);
            break;
        case CTRL_MODE_VELOCITY:
            u = clamp_i32(s->setpoint[j], -s->g.duty_max[j], s->g.duty_max[j]);
            break;
        default:
            u = 0;
            break;
        }

        /* Enclavamiento (SIEMPRE): no empujar contra un final presionado.
         * Convención: duty > 0 va hacia MAX, duty < 0 hacia MIN. */
        if (((limits & LIMIT_BIT_MIN(j)) && u < 0) || ((limits & LIMIT_BIT_MAX(j)) && u > 0)) {
            u = 0;
            s->integ[j] = 0;
        }

        if (s->stall_fault[j]) u = 0;
        stall_check(s, j, u, pos[j]);
        if (s->stall_fault[j]) {
            u = 0;
            flags |= STATUS_BIT_STALL;
        }
        out[j] = u;
    }
    if (s->watchdog_tripped) flags |= STATUS_BIT_WATCHDOG;
    return flags;
}

size_t ctrl_state_size(void)
{
    return sizeof(ctrl_state_t);
}

size_t debounce_state_size(void)
{
    return sizeof(debounce_t);
}
