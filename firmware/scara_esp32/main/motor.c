/*
 * motor.c — PWM con LEDC + dirección con IN1/IN2 del TB6612FNG.
 *
 * Tabla de verdad del TB6612 (datasheet Toshiba, "H-SW Control Function"):
 *   IN1 IN2 PWM | salida
 *    1   0   1  | CW  (gira en un sentido)
 *    0   1   1  | CCW (el otro)
 *    1   1   x  | freno (short brake)
 *    x   x   0  | freno (short brake) → el "apagado" del PWM también frena
 *    0   0   1  | stop libre (alta impedancia)
 * Como el tramo bajo del PWM frena (decaimiento lento), la velocidad es casi
 * proporcional al duty: buena linealidad para el PID.
 */
#include "motor.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "config.h"
#include "pins.h"

#define LEDC_MODE  LEDC_LOW_SPEED_MODE
#define LEDC_TIMER LEDC_TIMER_0
#define PWM_MAX_RAW ((1 << PWM_RES_BITS) - 1)

static const gpio_num_t s_pin_pwm[NUM_JOINTS] = PINS_M_PWM;
static const gpio_num_t s_pin_in1[NUM_JOINTS] = PINS_M_IN1;
static const gpio_num_t s_pin_in2[NUM_JOINTS] = PINS_M_IN2;
/* Un canal LEDC por motor. */
static const ledc_channel_t s_chan[NUM_JOINTS] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2};
static bool s_invert[NUM_JOINTS] = MOTOR_INVERT_INIT;

esp_err_t motor_init(void)
{
    /* Primero los pines de dirección a 1/1 (freno) para que no haya tirones al arrancar. */
    for (int j = 0; j < NUM_JOINTS; j++) {
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << s_pin_in1[j]) | (1ULL << s_pin_in2[j]),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&io));
        gpio_set_level(s_pin_in1[j], 1);
        gpio_set_level(s_pin_in2[j], 1);
    }

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = PWM_RES_BITS,
        .timer_num = LEDC_TIMER,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    for (int j = 0; j < NUM_JOINTS; j++) {
        ledc_channel_config_t ch = {
            .gpio_num = s_pin_pwm[j],
            .speed_mode = LEDC_MODE,
            .channel = s_chan[j],
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch));
    }
    return ESP_OK;
}

void motor_brake(int joint)
{
    if (joint < 0 || joint >= NUM_JOINTS) return;
    gpio_set_level(s_pin_in1[joint], 1);
    gpio_set_level(s_pin_in2[joint], 1);
    ledc_set_duty(LEDC_MODE, s_chan[joint], 0);
    ledc_update_duty(LEDC_MODE, s_chan[joint]);
}

void motor_brake_all(void)
{
    for (int j = 0; j < NUM_JOINTS; j++) motor_brake(j);
}

void motor_set(int joint, int32_t duty_permille)
{
    if (joint < 0 || joint >= NUM_JOINTS) return;
    if (duty_permille == 0) {
        motor_brake(joint);
        return;
    }
    if (duty_permille > DUTY_FULL_PERMILLE) duty_permille = DUTY_FULL_PERMILLE;
    if (duty_permille < -DUTY_FULL_PERMILLE) duty_permille = -DUTY_FULL_PERMILLE;
    if (s_invert[joint]) duty_permille = -duty_permille;

    const bool forward = duty_permille > 0;
    const uint32_t mag = (uint32_t)(forward ? duty_permille : -duty_permille);
    const uint32_t raw = (mag * PWM_MAX_RAW) / DUTY_FULL_PERMILLE;

    /* Dirección primero, luego duty: así nunca hay un instante con PWM alto y
     * la dirección vieja. */
    gpio_set_level(s_pin_in1[joint], forward ? 1 : 0);
    gpio_set_level(s_pin_in2[joint], forward ? 0 : 1);
    ledc_set_duty(LEDC_MODE, s_chan[joint], raw);
    ledc_update_duty(LEDC_MODE, s_chan[joint]);
}

void motor_set_invert(int joint, bool invert)
{
    if (joint < 0 || joint >= NUM_JOINTS) return;
    s_invert[joint] = invert;
}
