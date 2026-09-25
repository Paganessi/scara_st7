/*
 * limit_switches.c — 6 pulsadores NA, activo bajo (pull-up externo 2 kΩ, pulsador a GND).
 *
 * Debounce por conteo: un contacto mecánico "rebota" unos pocos ms al cerrarse.
 * Solo aceptamos un cambio si la lectura cruda se mantiene igual LIMIT_DEBOUNCE_N
 * veces seguidas (a 1 lectura/ms → 5 ms). Es corto comparado con lo que se mueve
 * la junta en ese tiempo y elimina falsos disparos.
 */
#include "limit_switches.h"

#include "driver/gpio.h"
#include "config.h"
#include "pins.h"

static const gpio_num_t s_pins[NUM_LIMITS] = PINS_LIMITS;
static debounce_t s_deb;               /* lógica del debounce en ctrl_logic.c */
static volatile uint8_t s_stable;       /* último estado aceptado */

uint8_t limits_read_raw(void)
{
    uint8_t mask = 0;
    for (int i = 0; i < NUM_LIMITS; i++) {
        /* Activo bajo: nivel 0 = presionado = bit en 1. */
        if (gpio_get_level(s_pins[i]) == 0) mask |= (uint8_t)(1u << i);
    }
    return mask;
}

esp_err_t limits_init(void)
{
    uint64_t mask = 0;
    for (int i = 0; i < NUM_LIMITS; i++) mask |= (1ULL << s_pins[i]);
    gpio_config_t io = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT,
        /* El pull-up externo de 2 kΩ manda; el interno (~45 kΩ) en paralelo no estorba
         * y deja el pin definido si algún día se suelta un cable. */
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    s_stable = limits_read_raw();
    debounce_init(&s_deb, s_stable);
    return err;
}

void limits_update(void)
{
    s_stable = debounce_update(&s_deb, limits_read_raw());
}

uint8_t limits_get(void)
{
    return s_stable;
}
