/*
 * pins.h — Mapa de pines de la baquela (SCARA Estación 7, Grupo 6-F).
 *
 * ÚNICO lugar del firmware donde aparecen números de GPIO.
 * Copia fiel del mapa de pines de la baquela. El mapa está SOLDADO: no se cambia aquí
 * sin cambiar la PCB (y eso no va a pasar).
 *
 * Índices de junta usados en todo el firmware: 0 = θ1 (hombro), 1 = θ2 (codo), 2 = Z.
 * (En el protocolo /scara/cmd el modo ZERO usa 1..3; la conversión se hace en control.c.)
 */
#pragma once

#include "driver/gpio.h"

#define NUM_JOINTS 3

/* ---------------- Encoders (cuadratura, llegan por divisor 3.3k/1.2k) ----------------
 * 34–39 son solo-entrada y sin pull-up interno: el divisor fija el nivel.
 * 36/39 tienen la errata de glitches con ADC/Wi-Fi: no usamos ninguno de los dos
 * y además el filtro de glitch del PCNT los limpia. */
#define PIN_ENC1_A GPIO_NUM_36 /* VP  — θ1 */
#define PIN_ENC1_B GPIO_NUM_39 /* VN  — θ1 */
#define PIN_ENC2_A GPIO_NUM_34 /* D34 — θ2 */
#define PIN_ENC2_B GPIO_NUM_35 /* D35 — θ2 */
#define PIN_ENC3_A GPIO_NUM_32 /* D32 — Z  */
#define PIN_ENC3_B GPIO_NUM_33 /* D33 — Z  */

/* ---------------- Motores → TB6612FNG ----------------
 * PWM = velocidad (LEDC), IN1/IN2 = dirección o freno.
 * GPIO12 y GPIO2 son pines de arranque: como SALIDAS hacia el TB6612 no molestan
 * (el TB6612 tiene pull-down interno, igual que lo que el ESP32 espera al arrancar). */
#define PIN_M1_PWM GPIO_NUM_25 /* Driver1 PWMA — θ1 */
#define PIN_M1_IN1 GPIO_NUM_26 /* Driver1 AIN1 */
#define PIN_M1_IN2 GPIO_NUM_27 /* Driver1 AIN2 */

#define PIN_M2_PWM GPIO_NUM_14 /* Driver1 PWMB — θ2 */
#define PIN_M2_IN1 GPIO_NUM_12 /* Driver1 BIN1 (strapping, solo salida) */
#define PIN_M2_IN2 GPIO_NUM_13 /* Driver1 BIN2 */

#define PIN_M3_PWM GPIO_NUM_16 /* Driver2 PWMA — Z (serigrafía RX2) */
#define PIN_M3_IN1 GPIO_NUM_4  /* Driver2 AIN1 */
#define PIN_M3_IN2 GPIO_NUM_2  /* Driver2 AIN2 (strapping; el LED azul parpadea con él) */

/* ---------------- Finales de carrera ----------------
 * Pulsador NA entre GPIO y GND, pull-up externo 2 kΩ a 3.3 V.
 * Suelto = HIGH, presionado = LOW (activo bajo). */
#define PIN_FC1_MIN GPIO_NUM_23 /* θ1 MIN */
#define PIN_FC1_MAX GPIO_NUM_22 /* θ1 MAX */
#define PIN_FC2_MIN GPIO_NUM_21 /* θ2 MIN */
#define PIN_FC2_MAX GPIO_NUM_19 /* θ2 MAX */
#define PIN_FC3_MIN GPIO_NUM_18 /* Z  MIN */
#define PIN_FC3_MAX GPIO_NUM_17 /* Z  MAX (serigrafía TX2) */

/* ---------------- Reservados / prohibidos ----------------
 * GPIO5  → servo del efector (aún no instalado).
 * GPIO1/3 → UART0 = USB = transporte micro-ROS. NUNCA tocar.
 * GPIO6–11 → flash interna. NUNCA tocar.
 * GPIO15 → libre. */
#define PIN_SERVO_RESERVED GPIO_NUM_5

/* Tablas por junta para que el resto del código itere con un índice. */
#define PINS_ENC_A   {PIN_ENC1_A, PIN_ENC2_A, PIN_ENC3_A}
#define PINS_ENC_B   {PIN_ENC1_B, PIN_ENC2_B, PIN_ENC3_B}
#define PINS_M_PWM   {PIN_M1_PWM, PIN_M2_PWM, PIN_M3_PWM}
#define PINS_M_IN1   {PIN_M1_IN1, PIN_M2_IN1, PIN_M3_IN1}
#define PINS_M_IN2   {PIN_M1_IN2, PIN_M2_IN2, PIN_M3_IN2}
/* Orden = orden de bits de /scara/limits: bit0 FC1_MIN, bit1 FC1_MAX, ... bit5 FC3_MAX */
#define PINS_LIMITS  {PIN_FC1_MIN, PIN_FC1_MAX, PIN_FC2_MIN, PIN_FC2_MAX, PIN_FC3_MIN, PIN_FC3_MAX}
#define NUM_LIMITS 6
