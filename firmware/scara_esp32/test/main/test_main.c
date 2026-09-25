/*
 * test_main.c — Firmware de prueba de la FASE 1 (sin micro-ROS).
 *
 * Objetivo: comprobar hardware y armar la TABLA DE SIGNOS antes de meter ROS:
 *   - cada motor se mueve en lazo abierto con pulsos cortos (duty y tiempo limitados)
 *   - el encoder de cada junta cuenta (girando a mano o con el motor)
 *   - cada final de carrera cambia su bit
 *
 * Reutiliza EXACTAMENTE los drivers del firmware real (../../main/motor.c, encoder.c,
 * limit_switches.c): lo que se valida aquí es lo mismo que correrá con micro-ROS.
 *
 * Seguridad (mientras los signos no están confirmados el enclavamiento "por dirección"
 * no es fiable, así que aquí se usa uno más conservador):
 *   - un pulso dura como máximo TEST_MS_MAX y el duty no pasa de TEST_DUTY_MAX
 *   - si CUALQUIER final de esa junta pasa de suelto a presionado durante el pulso → freno
 *   - "s" + Enter frena todo en cualquier momento
 *
 * Uso: idf.py -p /dev/ttyUSB0 flash monitor   y escribir comandos + Enter:
 *   m <j> <duty> <ms>  pulso en junta j (1..3), duty con signo en ‰, duración en ms
 *   s                  STOP (freno en todo)
 *   z <j>              poner a cero el contador de la junta j
 *   i <j> <m|e> <0|1>  invertir motor (m) o encoder (e) de la junta j (para probar signos)
 *   p                  pausar / reanudar la impresión periódica del estado
 *   h                  ayuda
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"

#include "config.h"
#include "encoder.h"
#include "limit_switches.h"
#include "motor.h"
#include "pins.h"

#define TEST_DUTY_MAX 400   /* ‰: tope de la prueba (primer movimiento: ≤ 250) */
#define TEST_MS_MAX   2000  /* ms: tope de duración de un pulso (primer movimiento: ≤ 500) */
#define PRINT_PERIOD_MS 200

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
/* Pulso en curso (lo escribe la consola, lo ejecuta la tarea de 1 kHz). */
static volatile int s_pulse_joint = -1;
static volatile int32_t s_pulse_duty;
static volatile int32_t s_pulse_ms_left;
static volatile bool s_pulse_new;
static volatile bool s_print_on = true;

/* Resultado del último pulso, para imprimirlo desde la tarea de consola. */
static volatile bool s_report_ready;
static volatile int s_rep_joint;
static volatile int32_t s_rep_duty, s_rep_ms, s_rep_delta;
static volatile uint8_t s_rep_hit;

static const char *lim_name[NUM_LIMITS] = {"FC1_MIN", "FC1_MAX", "FC2_MIN",
                                           "FC2_MAX", "FC3_MIN", "FC3_MAX"};

/* Tarea de 1 kHz: debounce de finales + ejecución segura del pulso. */
static void rt_task(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();
    int joint = -1;
    int32_t duty = 0, ms_done = 0, ms_left = 0, start_count = 0;
    uint8_t lim_at_start = 0;

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
        limits_update();
        const uint8_t lim = limits_get();

        portENTER_CRITICAL(&s_mux);
        const bool is_new = s_pulse_new;
        if (is_new) {
            s_pulse_new = false;
            if (joint >= 0) motor_brake(joint);
            joint = s_pulse_joint;
            duty = s_pulse_duty;
            ms_left = s_pulse_ms_left;
        }
        portEXIT_CRITICAL(&s_mux);

        if (is_new) {
            if (joint < 0) { /* comando "s" */
                motor_brake_all();
                continue;
            }
            ms_done = 0;
            start_count = encoder_get(joint);
            lim_at_start = lim;
        }
        if (joint < 0) continue;

        /* Freno si un final de ESTA junta pasa de suelto a presionado (flanco). */
        const uint8_t jmask = (uint8_t)(LIMIT_BIT_MIN(joint) | LIMIT_BIT_MAX(joint));
        const uint8_t new_hits = (uint8_t)(lim & ~lim_at_start & jmask);

        if (ms_left <= 0 || new_hits) {
            motor_brake(joint);
            s_rep_joint = joint;
            s_rep_duty = duty;
            s_rep_ms = ms_done;
            s_rep_delta = encoder_get(joint) - start_count;
            s_rep_hit = new_hits;
            s_report_ready = true;
            joint = -1;
            continue;
        }
        motor_set(joint, duty);
        ms_left--;
        ms_done++;
    }
}

static void print_status(void)
{
    const uint8_t lim = limits_get();
    printf("c1=%8ld c2=%8ld c3=%8ld | FC 1m1M 2m2M 3m3M = %d%d %d%d %d%d | mask=0x%02x\n",
           (long)encoder_get(0), (long)encoder_get(1), (long)encoder_get(2),
           !!(lim & 1), !!(lim & 2), !!(lim & 4), !!(lim & 8), !!(lim & 16), !!(lim & 32), lim);
}

static void print_help(void)
{
    printf("\n=== SCARA Fase 1: prueba de hardware (sin micro-ROS) ===\n"
           "  m <j> <duty> <ms>  pulso junta j=1..3, duty -%d..%d permil, ms<=%d\n"
           "  s                  STOP (freno)\n"
           "  z <j>              contador de junta j a 0\n"
           "  i <j> <m|e> <0|1>  invertir motor(m)/encoder(e) de la junta j\n"
           "  p                  pausar/reanudar impresion\n"
           "  h                  esta ayuda\n"
           "Convencion buscada: duty>0 => cuentas suben => va hacia el final MAX\n\n",
           TEST_DUTY_MAX, TEST_DUTY_MAX, TEST_MS_MAX);
}

static void printer_task(void *arg)
{
    (void)arg;
    uint8_t last_lim = 0xFF;
    TickType_t last_print = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (s_report_ready) {
            s_report_ready = false;
            printf(">> pulso junta %d: duty %ld permil, %ld ms -> delta cuentas = %ld%s\n",
                   s_rep_joint + 1, (long)s_rep_duty, (long)s_rep_ms, (long)s_rep_delta,
                   s_rep_hit ? "  (FRENADO POR FINAL)" : "");
            for (int i = 0; i < NUM_LIMITS; i++)
                if (s_rep_hit & (1u << i)) printf("   final activado: %s\n", lim_name[i]);
        }
        /* Cada cambio de finales se imprime al instante (para identificar cada pulsador). */
        const uint8_t lim = limits_get();
        if (lim != last_lim) {
            for (int i = 0; i < NUM_LIMITS; i++) {
                const uint8_t b = (uint8_t)(1u << i);
                if ((lim ^ last_lim) & b && last_lim != 0xFF)
                    printf("** %s %s\n", lim_name[i], (lim & b) ? "PRESIONADO" : "suelto");
            }
            last_lim = lim;
        }
        if (s_print_on && (xTaskGetTickCount() - last_print) >= pdMS_TO_TICKS(PRINT_PERIOD_MS)) {
            last_print = xTaskGetTickCount();
            print_status();
        }
    }
}

static void request_pulse(int joint, int32_t duty, int32_t ms)
{
    portENTER_CRITICAL(&s_mux);
    s_pulse_joint = joint;
    s_pulse_duty = duty;
    s_pulse_ms_left = ms;
    s_pulse_new = true;
    portEXIT_CRITICAL(&s_mux);
}

static void console_init(void)
{
    /* stdin/stdout por el driver de UART (bloqueante, con fin de línea CR del monitor). */
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 512, 0, 0, NULL, 0));
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    uart_vfs_dev_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CR);
    uart_vfs_dev_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
}

void app_main(void)
{
    ESP_ERROR_CHECK(motor_init()); /* freno primero */
    ESP_ERROR_CHECK(encoder_init());
    ESP_ERROR_CHECK(limits_init());
    console_init();

    xTaskCreatePinnedToCore(rt_task, "rt", 4096, NULL, configMAX_PRIORITIES - 2, NULL, 1);
    xTaskCreatePinnedToCore(printer_task, "print", 4096, NULL, 3, NULL, 0);
    print_help();

    char line[64];
    for (;;) {
        if (fgets(line, sizeof(line), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        line[strcspn(line, "\r\n")] = 0;
        if (line[0] == 0) continue;
        printf("> %s\n", line);

        int j = 0, d = 0, t = 0, v = 0;
        char which = 0;
        if (sscanf(line, "m %d %d %d", &j, &d, &t) == 3) {
            if (j < 1 || j > NUM_JOINTS) { printf("junta invalida\n"); continue; }
            if (d > TEST_DUTY_MAX) d = TEST_DUTY_MAX;
            if (d < -TEST_DUTY_MAX) d = -TEST_DUTY_MAX;
            if (t > TEST_MS_MAX) t = TEST_MS_MAX;
            if (t < 1) t = 1;
            printf("pulso junta %d: %d permil, %d ms\n", j, d, t);
            request_pulse(j - 1, d, t);
        } else if (line[0] == 's') {
            request_pulse(-1, 0, 0);
            printf("STOP\n");
        } else if (sscanf(line, "z %d", &j) == 1 && j >= 1 && j <= NUM_JOINTS) {
            encoder_set(j - 1, 0);
            printf("contador junta %d = 0\n", j);
        } else if (sscanf(line, "i %d %c %d", &j, &which, &v) == 3 && j >= 1 && j <= NUM_JOINTS) {
            if (which == 'm') motor_set_invert(j - 1, v != 0);
            else if (which == 'e') encoder_set_invert(j - 1, v != 0);
            printf("junta %d: invertir %s = %d\n", j, which == 'm' ? "motor" : "encoder", v != 0);
        } else if (line[0] == 'p') {
            s_print_on = !s_print_on;
        } else if (line[0] == 'h') {
            print_help();
        } else {
            printf("comando no reconocido (h = ayuda)\n");
        }
    }
}
