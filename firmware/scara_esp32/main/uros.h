/*
 * uros.h — Nodo micro-ROS "scara_esp32" (core 0).
 */
#pragma once

#include "esp_err.h"

/* Registra el transporte serial y lanza la tarea micro-ROS (con reconexión). */
esp_err_t uros_start(void);
