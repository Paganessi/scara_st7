/*
 * test_ctrl_logic.c — Pruebas unitarias de la lógica del firmware, en el PC con gcc.
 *
 *   cd firmware/scara_esp32/host_test && make test
 *
 * Prueban ctrl_logic.c (el MISMO archivo que corre en el ESP32) sin hardware:
 * debounce, watchdog, modos, enclavamiento con finales, PID (P, I con anti-windup,
 * D sobre la medición, zona muerta, saturación), bloqueo (stall), ZERO, configuración
 * en caliente y un lazo cerrado contra una planta simulada.
 *
 * Mini-framework propio (sin dependencias): CHECK() cuenta fallos y sigue; al final
 * el programa devuelve 1 si algo falló (así `make test` falla en CI).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ctrl_logic.h"

static int g_checks, g_fails;
#define CHECK(cond)                                                                  \
    do {                                                                             \
        g_checks++;                                                                  \
        if (!(cond)) {                                                               \
            g_fails++;                                                               \
            printf("  FALLA %s:%d: %s\n", __FILE__, __LINE__, #cond);                \
        }                                                                            \
    } while (0)
#define CHECK_EQ(a, b)                                                               \
    do {                                                                             \
        long long _a = (long long)(a), _b = (long long)(b);                          \
        g_checks++;                                                                  \
        if (_a != _b) {                                                              \
            g_fails++;                                                               \
            printf("  FALLA %s:%d: %s == %s  (%lld != %lld)\n", __FILE__, __LINE__, #a, \
                   #b, _a, _b);                                                      \
        }                                                                            \
    } while (0)
#define RUN(t)                                     \
    do {                                           \
        int f0 = g_fails;                          \
        t();                                       \
        printf("%s %s\n", g_fails == f0 ? "[ OK ]" : "[FAIL]", #t); \
    } while (0)

static const int32_t ZERO3[NJ] = {0, 0, 0};

/* Avanza n ms con posición y finales fijos; devuelve el último byte de estado. */
static uint8_t steps(ctrl_state_t *s, const int32_t pos[NJ], uint8_t lim, int n, int32_t out[NJ])
{
    uint8_t f = 0;
    for (int i = 0; i < n; i++) f = ctrl_step(s, pos, lim, out);
    return f;
}

/* Igual que steps() pero re-enviando el comando cada 50 ms (keepalive del bridge). */
static void steps_ka(ctrl_state_t *s, int32_t mode, int32_t v1, int32_t v2, int32_t v3,
                     const int32_t pos[NJ], int n, int32_t out[NJ])
{
    for (int i = 0; i < n; i++) {
        if (i % 50 == 0) ctrl_command(s, mode, v1, v2, v3);
        ctrl_step(s, pos, 0, out);
    }
}

/* ---------------------------------------------------------------- debounce */
static void test_debounce_filtra_rebotes(void)
{
    debounce_t d;
    debounce_init(&d, 0x00);
    /* Un rebote de 1–4 ms no cambia el estado */
    for (int i = 0; i < LIMIT_DEBOUNCE_N - 1; i++) CHECK_EQ(debounce_update(&d, 0x01), 0x00);
    CHECK_EQ(debounce_update(&d, 0x00), 0x00);
    /* N lecturas iguales seguidas sí lo cambian */
    for (int i = 0; i < LIMIT_DEBOUNCE_N - 1; i++) CHECK_EQ(debounce_update(&d, 0x01), 0x00);
    CHECK_EQ(debounce_update(&d, 0x01), 0x01);
    /* Y al soltar igual */
    for (int i = 0; i < LIMIT_DEBOUNCE_N - 1; i++) CHECK_EQ(debounce_update(&d, 0x00), 0x01);
    CHECK_EQ(debounce_update(&d, 0x00), 0x00);
}

static void test_debounce_estado_inicial(void)
{
    debounce_t d;
    debounce_init(&d, 0x24);
    CHECK_EQ(debounce_update(&d, 0x24), 0x24);
}

/* ---------------------------------------------------------------- watchdog / modos */
static void test_arranca_en_stop_con_watchdog(void)
{
    ctrl_state_t s;
    int32_t out[NJ];
    ctrl_init(&s, ZERO3);
    const uint8_t f = ctrl_step(&s, ZERO3, 0, out);
    CHECK(f & STATUS_BIT_WATCHDOG);
    CHECK_EQ(out[0], 0);
    CHECK_EQ(s.mode, CTRL_MODE_STOP);
}

static void test_watchdog_para_a_los_500ms(void)
{
    ctrl_state_t s;
    int32_t out[NJ];
    ctrl_init(&s, ZERO3);
    ctrl_command(&s, CTRL_MODE_VELOCITY, 200, 0, 0);
    uint8_t f = steps(&s, ZERO3, 0, WATCHDOG_MS, out);
    CHECK_EQ(out[0], 200);                 /* a los 500 ms todavía se mueve */
    CHECK(!(f & STATUS_BIT_WATCHDOG));
    f = steps(&s, ZERO3, 0, 1, out);
    CHECK_EQ(out[0], 0);                   /* a los 501 ms: STOP */
    CHECK(f & STATUS_BIT_WATCHDOG);
    CHECK_EQ(s.mode, CTRL_MODE_STOP);
}

static void test_keepalive_mantiene_vivo(void)
{
    ctrl_state_t s;
    int32_t out[NJ];
    ctrl_init(&s, ZERO3);
    for (int k = 0; k < 20; k++) {         /* 20 × 50 ms = 1 s con keepalive a 20 Hz */
        ctrl_command(&s, CTRL_MODE_VELOCITY, 150, 0, 0);
        steps(&s, ZERO3, 0, 50, out);
        CHECK_EQ(out[0], 150);
    }
}

static void test_config_cuenta_como_keepalive_y_no_cambia_modo(void)
{
    ctrl_state_t s;
    int32_t out[NJ];
    ctrl_init(&s, ZERO3);
    ctrl_command(&s, CTRL_MODE_VELOCITY, 150, 0, 0);
    steps(&s, ZERO3, 0, 400, out);
    ctrl_command(&s, CTRL_MODE_SET_KD, 1, 5, 0);
    steps(&s, ZERO3, 0, 400, out);         /* 800 ms desde el VELOCITY, 400 desde la config */
    CHECK_EQ(s.mode, CTRL_MODE_VELOCITY);
    CHECK_EQ(out[0], 150);
}

static void test_modo_desconocido_es_stop(void)
{
    ctrl_state_t s;
    int32_t out[NJ];
    ctrl_init(&s, ZERO3);
    ctrl_command(&s, CTRL_MODE_VELOCITY, 300, 300, 300);
    ctrl_command(&s, 42, 300, 300, 300);
    steps(&s, ZERO3, 0, 1, out);
    CHECK_EQ(s.mode, CTRL_MODE_STOP);
    CHECK_EQ(out[0], 0);
    CHECK_EQ(out[2], 0);
}

static void test_velocity_satura_a_duty_max(void)
{
    ctrl_state_t s;
    int32_t out[NJ];
    ctrl_init(&s, ZERO3);
    ctrl_command(&s, CTRL_MODE_VELOCITY, 1000, -1000, 100);
    steps(&s, ZERO3, 0, 1, out);
    CHECK_EQ(out[0], 600);
    CHECK_EQ(out[1], -600);
    CHECK_EQ(out[2], 100);
}

/* ---------------------------------------------------------------- enclavamiento */
static void test_enclavamiento_min_bloquea_solo_hacia_min(void)
{
    ctrl_state_t s;
    int32_t out[NJ];
    ctrl_init(&s, ZERO3);
    const uint8_t fc1_min = LIMIT_BIT_MIN(0);
    ctrl_command(&s, CTRL_MODE_VELOCITY, -300, 0, 0);
    steps(&s, ZERO3, fc1_min, 1, out);
    CHECK_EQ(out[0], 0);                   /* hacia MIN con MIN presionado: bloqueado */
    ctrl_command(&s, CTRL_MODE_VELOCITY, 300, 0, 0);
    steps(&s, ZERO3, fc1_min, 1, out);
    CHECK_EQ(out[0], 300);                 /* alejarse de MIN: permitido */
}

static void test_enclavamiento_max_y_otras_juntas(void)
{
    ctrl_state_t s;
    int32_t out[NJ];
    ctrl_init(&s, ZERO3);
    const uint8_t fc3_max = LIMIT_BIT_MAX(2);
    ctrl_command(&s, CTRL_MODE_VELOCITY, 300, 300, 300);
    const uint8_t f = steps(&s, ZERO3, fc3_max, 1, out);
    CHECK_EQ(out[0], 300);                 /* el final de Z no afecta a θ1 */
    CHECK_EQ(out[1], 300);
    CHECK_EQ(out[2], 0);                   /* Z hacia MAX con MAX presionado */
    CHECK_EQ(f & LIMITS_MASK, fc3_max);    /* el bit viaja en /scara/limits */
}

static void test_enclavamiento_en_position(void)
{
    ctrl_state_t s;
    int32_t out[NJ];
    ctrl_init(&s, ZERO3);
    ctrl_command(&s, CTRL_MODE_POSITION, -1000, 0, 0);   /* meta hacia MIN */
    steps(&s, ZERO3, LIMIT_BIT_MIN(0), 1, out);
    CHECK_EQ(out[0], 0);
}

/* ---------------------------------------------------------------- PID */
static void test_pid_proporcional(void)
{
    ctrl_state_t s;
    int32_t out[NJ];
    ctrl_init(&s, ZERO3);                                /* KP = 1 ‰/cuenta */
    ctrl_command(&s, CTRL_MODE_POSITION, 200, -100, 0);
    steps(&s, ZERO3, 0, 1, out);
    CHECK_EQ(out[0], 200);
    CHECK_EQ(out[1], -100);
    CHECK_EQ(out[2], 0);
}

static void test_pid_banda_de_llegada(void)
{
    ctrl_state_t s;
    int32_t out[NJ];
    ctrl_init(&s, ZERO3);
    ctrl_command(&s, CTRL_MODE_POSITION, POS_TOLERANCE_COUNTS, -POS_TOLERANCE_COUNTS, 0);
    steps(&s, ZERO3, 0, 1, out);
    CHECK_EQ(out[0], 0);
    CHECK_EQ(out[1], 0);
    ctrl_command(&s, CTRL_MODE_POSITION, POS_TOLERANCE_COUNTS + 1, 0, 0);
    steps(&s, ZERO3, 0, 1, out);
    CHECK(out[0] > 0);
}

static void test_pid_satura(void)
{
    ctrl_state_t s;
    int32_t out[NJ];
    ctrl_init(&s, ZERO3);
    ctrl_command(&s, CTRL_MODE_POSITION, 100000, -100000, 0);
    steps(&s, ZERO3, 0, 1, out);
    CHECK_EQ(out[0], 600);
    CHECK_EQ(out[1], -600);
}

static void test_pid_zona_muerta(void)
{
    ctrl_state_t s;
    int32_t out[NJ];
    ctrl_init(&s, ZERO3);
    ctrl_command(&s, CTRL_MODE_SET_DUTY, 1, 120, 600);  /* duty_min 120 en θ1 */
    ctrl_command(&s, CTRL_MODE_POSITION, 10, 0, 0);
    steps(&s, ZERO3, 0, 1, out);
    CHECK_EQ(out[0], 10 + 120);
    ctrl_command(&s, CTRL_MODE_POSITION, -10, 0, 0);
    steps(&s, ZERO3, 0, 1, out);
    CHECK_EQ(out[0], -10 - 120);
}

static void test_pid_integral_y_antiwindup(void)
{
    ctrl_state_t s;
    int32_t out[NJ];
    ctrl_init(&s, ZERO3);
    ctrl_command(&s, CTRL_MODE_SET_GAINS, 1, 0, 1000);  /* solo I: 1 ‰/(cuenta·s) */
    steps_ka(&s, CTRL_MODE_POSITION, 100, 0, 0, ZERO3, 1000, out); /* 1 s, e=100 → 100 ‰ */
    CHECK(out[0] >= 95 && out[0] <= 101);
    steps_ka(&s, CTRL_MODE_POSITION, 100, 0, 0, ZERO3, 10000, out); /* largo: manda el clamp */
    CHECK_EQ(out[0], I_TERM_MAX_PERMILLE);
    /* Cambio de modo borra el integrador */
    ctrl_command(&s, CTRL_MODE_STOP, 0, 0, 0);
    ctrl_command(&s, CTRL_MODE_POSITION, 100, 0, 0);
    steps(&s, ZERO3, 0, 1, out);
    CHECK(out[0] <= 1);
}

static void test_pid_no_integra_saturado(void)
{
    ctrl_state_t s;
    int32_t out[NJ];
    ctrl_init(&s, ZERO3);
    ctrl_command(&s, CTRL_MODE_SET_GAINS, 1, 1000, 1000);  /* P fuerte + I */
    /* 250 ms (< STALL_TIME_MS: con el encoder quieto más tiempo saltaría el bloqueo) */
    steps_ka(&s, CTRL_MODE_POSITION, 5000, 0, 0, ZERO3, 250, out); /* P solo ya satura */
    CHECK_EQ(out[0], 600);
    CHECK_EQ(s.integ[0], 0);                               /* integración condicional */
}

static void test_pid_derivada_sobre_medicion(void)
{
    ctrl_state_t s;
    int32_t out[NJ];
    int32_t pos[NJ] = {0, 0, 0};
    ctrl_init(&s, pos);
    ctrl_command(&s, CTRL_MODE_SET_GAINS, 1, 0, 0);        /* sin P ni I */
    ctrl_command(&s, CTRL_MODE_SET_KD, 1, 1000, 0);        /* D = 1 ‰ por cuenta/s */
    ctrl_command(&s, CTRL_MODE_POSITION, 3000, 0, 0);      /* salto de meta, quieto */
    steps(&s, pos, 0, 1, out);
    CHECK_EQ(out[0], 0);                                   /* sin "patadón" */
    /* Moviéndose a +100 cuentas/s (1 cuenta cada 10 ms): D se opone */
    for (int k = 0; k < 50; k++) {
        if (k % 10 == 0) pos[0]++;
        ctrl_step(&s, pos, 0, out);
    }
    CHECK(out[0] < 0 && out[0] >= -110);
}

/* ---------------------------------------------------------------- bloqueo */
static void test_stall_corta_y_se_limpia_con_stop(void)
{
    ctrl_state_t s;
    int32_t out[NJ];
    ctrl_init(&s, ZERO3);
    ctrl_command(&s, CTRL_MODE_VELOCITY, 500, 0, 0);
    uint8_t f = 0;
    for (int k = 0; k < 8; k++) {              /* 400 ms con keepalive, encoder quieto */
        ctrl_command(&s, CTRL_MODE_VELOCITY, 500, 0, 0);
        f = steps(&s, ZERO3, 0, 50, out);
    }
    CHECK(f & STATUS_BIT_STALL);
    CHECK_EQ(out[0], 0);
    ctrl_command(&s, CTRL_MODE_VELOCITY, 500, 0, 0);   /* mismo modo: sigue cortado */
    steps(&s, ZERO3, 0, 1, out);
    CHECK_EQ(out[0], 0);
    ctrl_command(&s, CTRL_MODE_STOP, 0, 0, 0);          /* STOP limpia la falla */
    f = steps(&s, ZERO3, 0, 1, out);
    CHECK(!(f & STATUS_BIT_STALL));
}

static void test_stall_no_salta_si_se_mueve(void)
{
    ctrl_state_t s;
    int32_t out[NJ];
    int32_t pos[NJ] = {0, 0, 0};
    ctrl_init(&s, pos);
    uint8_t f = 0;
    for (int k = 0; k < 1000; k++) {
        if (k % 50 == 0) ctrl_command(&s, CTRL_MODE_VELOCITY, 500, 0, 0);
        pos[0] += 1;                               /* 1000 cuentas/s */
        f = ctrl_step(&s, pos, 0, out);
    }
    CHECK(!(f & STATUS_BIT_STALL));
    CHECK_EQ(out[0], 500);
}

static void test_stall_no_salta_con_duty_bajo(void)
{
    ctrl_state_t s;
    int32_t out[NJ];
    ctrl_init(&s, ZERO3);
    uint8_t f = 0;
    for (int k = 0; k < 20; k++) {
        ctrl_command(&s, CTRL_MODE_VELOCITY, STALL_DUTY_PERMILLE - 1, 0, 0);
        f = steps(&s, ZERO3, 0, 50, out);
    }
    CHECK(!(f & STATUS_BIT_STALL));
}

/* ---------------------------------------------------------------- ZERO y configuración */
static void test_zero_devuelve_efecto_y_queda_en_stop(void)
{
    ctrl_state_t s;
    ctrl_init(&s, ZERO3);
    ctrl_command(&s, CTRL_MODE_POSITION, 100, 0, 0);
    ctrl_effect_t e = ctrl_command(&s, CTRL_MODE_ZERO, 2, -2489, 0);
    CHECK(e.zero);
    CHECK_EQ(e.joint, 1);
    CHECK_EQ(e.value, -2489);
    CHECK_EQ(s.mode, CTRL_MODE_STOP);
    e = ctrl_command(&s, CTRL_MODE_ZERO, 7, 0, 0);       /* junta inválida: se ignora */
    CHECK(!e.zero);
}

static void test_set_duty_tope_duro_y_deshabilitar(void)
{
    ctrl_state_t s;
    int32_t out[NJ];
    ctrl_init(&s, ZERO3);
    ctrl_command(&s, CTRL_MODE_SET_DUTY, 1, 0, 1000);
    CHECK_EQ(s.g.duty_max[0], DUTY_MAX_HARD);
    ctrl_command(&s, CTRL_MODE_SET_DUTY, 2, 0, 0);       /* J2 "deshabilitada" */
    ctrl_command(&s, CTRL_MODE_VELOCITY, 900, 900, 0);
    steps(&s, ZERO3, 0, 1, out);
    CHECK_EQ(out[0], DUTY_MAX_HARD);
    CHECK_EQ(out[1], 0);
    ctrl_command(&s, CTRL_MODE_POSITION, 0, 5000, 0);
    steps(&s, ZERO3, 0, 1, out);
    CHECK_EQ(out[1], 0);
}

/* ---------------------------------------------------------------- lazo cerrado */
/* Planta simple: motor de primer orden (τ = 30 ms) + integrador, con zona muerta.
 * Velocidad máx. 10 667 cuentas/s a duty 1000 ‰ (Pololu 37D a 12 V). */
static double plant_step(double *x, double *v, int32_t duty, int32_t deadzone)
{
    const double dt = 0.001, tau = 0.03, vmax = 10667.0;
    double eff = 0.0;
    if (duty > deadzone) eff = (duty - deadzone) / (1000.0 - deadzone);
    if (duty < -deadzone) eff = (duty + deadzone) / (1000.0 - deadzone);
    *v += (vmax * eff - *v) * dt / tau;
    if (duty == 0) *v *= 0.8;               /* freno activo */
    *x += *v * dt;
    return *x;
}

static void test_lazo_cerrado_llega_sin_oscilar(void)
{
    ctrl_state_t s;
    int32_t out[NJ];
    int32_t pos[NJ] = {0, 0, 0};
    double x = 0, v = 0;
    ctrl_init(&s, pos);
    ctrl_command(&s, CTRL_MODE_SET_DUTY, 1, 80, 600);    /* compensa la zona muerta */
    const int32_t target = 600;                          /* ≈ 34° de θ1 */
    int crossings = 0, prev_sign = 1;
    int32_t peak = 0;
    for (int ms = 0; ms < 2000; ms++) {
        if (ms % 20 == 0) ctrl_command(&s, CTRL_MODE_POSITION, target, 0, 0);
        ctrl_step(&s, pos, 0, out);
        pos[0] = (int32_t)plant_step(&x, &v, out[0], 70);
        if (pos[0] > peak) peak = pos[0];
        const int32_t e = target - pos[0];
        const int sg = e > POS_TOLERANCE_COUNTS ? 1 : (e < -POS_TOLERANCE_COUNTS ? -1 : 0);
        if (sg != 0 && sg != prev_sign) crossings++;
        if (sg != 0) prev_sign = sg;
    }
    const int32_t err = target - pos[0];
    printf("    lazo cerrado: final=%d error=%d pico=%d cruces=%d\n", pos[0], err, peak,
           crossings);
    CHECK(err >= -10 && err <= 10);         /* criterio Fase 3: dentro de ±N (N = 10) */
    CHECK(crossings <= 1);                  /* sin oscilar */
    CHECK(peak <= target + target / 10);    /* sobreimpulso ≤ 10 % */
}

int main(void)
{
    printf("=== Pruebas de ctrl_logic.c (sizeof ctrl_state_t = %zu B) ===\n", ctrl_state_size());
    RUN(test_debounce_filtra_rebotes);
    RUN(test_debounce_estado_inicial);
    RUN(test_arranca_en_stop_con_watchdog);
    RUN(test_watchdog_para_a_los_500ms);
    RUN(test_keepalive_mantiene_vivo);
    RUN(test_config_cuenta_como_keepalive_y_no_cambia_modo);
    RUN(test_modo_desconocido_es_stop);
    RUN(test_velocity_satura_a_duty_max);
    RUN(test_enclavamiento_min_bloquea_solo_hacia_min);
    RUN(test_enclavamiento_max_y_otras_juntas);
    RUN(test_enclavamiento_en_position);
    RUN(test_pid_proporcional);
    RUN(test_pid_banda_de_llegada);
    RUN(test_pid_satura);
    RUN(test_pid_zona_muerta);
    RUN(test_pid_integral_y_antiwindup);
    RUN(test_pid_no_integra_saturado);
    RUN(test_pid_derivada_sobre_medicion);
    RUN(test_stall_corta_y_se_limpia_con_stop);
    RUN(test_stall_no_salta_si_se_mueve);
    RUN(test_stall_no_salta_con_duty_bajo);
    RUN(test_zero_devuelve_efecto_y_queda_en_stop);
    RUN(test_set_duty_tope_duro_y_deshabilitar);
    RUN(test_lazo_cerrado_llega_sin_oscilar);
    printf("=== %d comprobaciones, %d fallas ===\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
