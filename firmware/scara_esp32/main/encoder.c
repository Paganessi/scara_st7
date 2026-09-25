/*
 * encoder.c — Cuadratura x4 con PCNT (driver nuevo "driver/pulse_cnt.h" de ESP-IDF v5).
 *
 * Idea física: las señales A y B están desfasadas 90°. Mirando QUÉ flanco llega y en
 * QUÉ nivel está la otra señal se sabe el sentido de giro. Contando los flancos de
 * subida y bajada de A y de B obtenemos 4 cuentas por ciclo → "x4" → 64 cuentas por
 * vuelta del motor en el Pololu 37D.
 *
 * Con 2 canales por unidad PCNT:
 *   canal A: cuenta en los flancos de A, mira el nivel de B
 *   canal B: cuenta en los flancos de B, mira el nivel de A
 * Configuración tomada del ejemplo oficial de ESP-IDF
 * (examples/peripherals/pcnt/rotary_encoder, Apache-2.0).
 *
 * El contador hardware es de 16 bits. Con accum_count=1 y watch points en los
 * límites, el driver suma el desborde en software → la cuenta total es de 32 bits
 * (el eje Z con husillo puede pasar de 32767 cuentas en su carrera).
 */
#include "encoder.h"

#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "config.h"
#include "pins.h"

static const char *TAG = "encoder";

static pcnt_unit_handle_t s_unit[NUM_JOINTS];
static const gpio_num_t s_pin_a[NUM_JOINTS] = PINS_ENC_A;
static const gpio_num_t s_pin_b[NUM_JOINTS] = PINS_ENC_B;
static bool s_invert[NUM_JOINTS] = ENCODER_INVERT_INIT;
/* Posición = signo·crudo + offset. El offset permite el "ZERO" sin tocar el hardware. */
static volatile int32_t s_offset[NUM_JOINTS];

static int32_t read_raw(int joint)
{
    int raw = 0;
    pcnt_unit_get_count(s_unit[joint], &raw);
    return (int32_t)raw;
}

esp_err_t encoder_init(void)
{
    for (int j = 0; j < NUM_JOINTS; j++) {
        pcnt_unit_config_t unit_cfg = {
            .high_limit = ENC_PCNT_HIGH,
            .low_limit = ENC_PCNT_LOW,
            .flags.accum_count = 1,
        };
        ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &s_unit[j]));

        pcnt_glitch_filter_config_t filt = {.max_glitch_ns = ENC_GLITCH_NS};
        ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_unit[j], &filt));

        pcnt_chan_config_t ca = {.edge_gpio_num = s_pin_a[j], .level_gpio_num = s_pin_b[j]};
        pcnt_chan_config_t cb = {.edge_gpio_num = s_pin_b[j], .level_gpio_num = s_pin_a[j]};
        pcnt_channel_handle_t ch_a, ch_b;
        ESP_ERROR_CHECK(pcnt_new_channel(s_unit[j], &ca, &ch_a));
        ESP_ERROR_CHECK(pcnt_new_channel(s_unit[j], &cb, &ch_b));

        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE));
        ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE));
        ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

        /* Watch points en los límites: necesarios para que accum_count funcione. */
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_unit[j], ENC_PCNT_HIGH));
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_unit[j], ENC_PCNT_LOW));

        ESP_ERROR_CHECK(pcnt_unit_enable(s_unit[j]));
        ESP_ERROR_CHECK(pcnt_unit_clear_count(s_unit[j]));
        ESP_ERROR_CHECK(pcnt_unit_start(s_unit[j]));
        s_offset[j] = 0;
    }
    ESP_LOGI(TAG, "3 encoders PCNT x4 listos (filtro %d ns)", ENC_GLITCH_NS);
    return ESP_OK;
}

int32_t encoder_get(int joint)
{
    if (joint < 0 || joint >= NUM_JOINTS) return 0;
    const int32_t raw = read_raw(joint);
    return (s_invert[joint] ? -raw : raw) + s_offset[joint];
}

void encoder_set(int joint, int32_t value)
{
    if (joint < 0 || joint >= NUM_JOINTS) return;
    /* No limpiamos el hardware (perderíamos el acumulado del driver a medias);
     * movemos el offset para que la lectura actual valga 'value'. */
    const int32_t raw = read_raw(joint);
    s_offset[joint] = value - (s_invert[joint] ? -raw : raw);
}

void encoder_set_invert(int joint, bool invert)
{
    if (joint < 0 || joint >= NUM_JOINTS) return;
    const int32_t pos = encoder_get(joint);
    s_invert[joint] = invert;
    encoder_set(joint, pos);
}
