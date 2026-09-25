/*
 * uros.c — Nodo micro-ROS "scara_esp32" en el core 0.
 *
 * Contrato:
 *   pub /scara/enc_counts  std_msgs/Int32MultiArray  [c1, c2, c3]         50 Hz
 *   pub /scara/limits      std_msgs/UInt8            bitmask finales      50 Hz
 *   sub /scara/cmd         std_msgs/Int32MultiArray  [modo, v1, v2, v3]   ≥20 Hz
 *
 * QoS: todo BEST_EFFORT. Un publisher "reliable" de micro-ROS espera la confirmación
 * del agente antes de seguir (bloquea hasta 1 s si algo se pierde); a 50 Hz preferimos
 * perder un dato viejo que frenar el nodo. Del lado ROS hay que suscribirse con QoS
 * sensor_data (best effort) o no hay "match".
 *
 * Reconexión: máquina de estados basada en el ejemplo "ping_pong"/"reconnection" de
 * micro-ROS (rmw_uros_ping_agent). Si el agente se cae, se destruyen las entidades y
 * se vuelve a esperar; mientras tanto el watchdog de control.c deja el robot en STOP.
 */
#include "uros.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/u_int8.h>

#include "config.h"
#include "control.h"
#include "esp32_serial_transport.h"

static const char *TAG = "uros";

#define CMD_CAPACITY 8 /* el contrato usa 4; margen por si llega algo más largo */

/* ---------- Entidades micro-ROS (estáticas: nada de malloc en caliente) ---------- */
static rcl_allocator_t s_allocator;
static rclc_support_t s_support;
static rcl_node_t s_node;
static rcl_publisher_t s_pub_counts;
static rcl_publisher_t s_pub_limits;
static rcl_subscription_t s_sub_cmd;
static rcl_timer_t s_timer;
static rclc_executor_t s_executor;

static std_msgs__msg__Int32MultiArray s_msg_counts;
static int32_t s_counts_buf[NUM_JOINTS];
static std_msgs__msg__UInt8 s_msg_limits;
static std_msgs__msg__Int32MultiArray s_msg_cmd;
static int32_t s_cmd_buf[CMD_CAPACITY];

static size_t s_uart_port = UART_NUM_0;

typedef enum { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } agent_state_t;

/* Arma un Int32MultiArray sobre un buffer estático (layout vacío). */
static void init_multiarray(std_msgs__msg__Int32MultiArray *m, int32_t *buf, size_t cap, size_t size)
{
    memset(m, 0, sizeof(*m));
    m->data.data = buf;
    m->data.capacity = cap;
    m->data.size = size;
    m->layout.dim.data = NULL;
    m->layout.dim.capacity = 0;
    m->layout.dim.size = 0;
    m->layout.data_offset = 0;
}

/* 50 Hz: publicar la foto del estado que deja la tarea de control. */
static void timer_cb(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) return;
    uint8_t flags;
    control_get_status(s_counts_buf, &flags);
    s_msg_limits.data = flags;
    (void)rcl_publish(&s_pub_counts, &s_msg_counts, NULL);
    (void)rcl_publish(&s_pub_limits, &s_msg_limits, NULL);
}

/* Llega /scara/cmd: se lo pasamos tal cual a control.c (él valida y aplica). */
static void cmd_cb(const void *msgin)
{
    const std_msgs__msg__Int32MultiArray *m = (const std_msgs__msg__Int32MultiArray *)msgin;
    if (m->data.size < 1) return;
    int32_t f[4] = {0, 0, 0, 0};
    for (size_t i = 0; i < 4 && i < m->data.size; i++) f[i] = m->data.data[i];
    control_command(f[0], f[1], f[2], f[3]);
}

static bool create_entities(void)
{
    s_allocator = rcl_get_default_allocator();
    if (rclc_support_init(&s_support, 0, NULL, &s_allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&s_node, "scara_esp32", "", &s_support) != RCL_RET_OK) return false;

    if (rclc_publisher_init_best_effort(&s_pub_counts, &s_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
            "/scara/enc_counts") != RCL_RET_OK) return false;
    if (rclc_publisher_init_best_effort(&s_pub_limits, &s_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
            "/scara/limits") != RCL_RET_OK) return false;
    if (rclc_subscription_init_best_effort(&s_sub_cmd, &s_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
            "/scara/cmd") != RCL_RET_OK) return false;

    if (rclc_timer_init_default2(&s_timer, &s_support, RCL_MS_TO_NS(PUBLISH_PERIOD_MS),
                                 timer_cb, true) != RCL_RET_OK) return false;

    s_executor = rclc_executor_get_zero_initialized_executor();
    if (rclc_executor_init(&s_executor, &s_support.context, 2, &s_allocator) != RCL_RET_OK) return false;
    if (rclc_executor_add_timer(&s_executor, &s_timer) != RCL_RET_OK) return false;
    if (rclc_executor_add_subscription(&s_executor, &s_sub_cmd, &s_msg_cmd, &cmd_cb,
                                       ON_NEW_DATA) != RCL_RET_OK) return false;
    return true;
}

static void destroy_entities(void)
{
    /* Si el agente ya no está, no esperar respuesta al destruir la sesión. */
    rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&s_support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

    (void)rcl_publisher_fini(&s_pub_counts, &s_node);
    (void)rcl_publisher_fini(&s_pub_limits, &s_node);
    (void)rcl_subscription_fini(&s_sub_cmd, &s_node);
    (void)rcl_timer_fini(&s_timer);
    (void)rclc_executor_fini(&s_executor);
    (void)rcl_node_fini(&s_node);
    (void)rclc_support_fini(&s_support);
}

static void uros_task(void *arg)
{
    (void)arg;
    agent_state_t state = WAITING_AGENT;
    TickType_t last_ping = 0;

    for (;;) {
        switch (state) {
        case WAITING_AGENT:
            /* Un ping cada 500 ms hasta que el agente conteste. */
            state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ? AGENT_AVAILABLE : WAITING_AGENT;
            if (state == WAITING_AGENT) vTaskDelay(pdMS_TO_TICKS(500));
            break;
        case AGENT_AVAILABLE:
            if (create_entities()) {
                ESP_LOGI(TAG, "conectado al agente");
                state = AGENT_CONNECTED;
                last_ping = xTaskGetTickCount();
            } else {
                destroy_entities();
                state = WAITING_AGENT;
            }
            break;
        case AGENT_CONNECTED:
            /* Cada 1 s confirmamos que el agente sigue vivo (3 intentos de 50 ms). */
            if ((xTaskGetTickCount() - last_ping) > pdMS_TO_TICKS(1000)) {
                last_ping = xTaskGetTickCount();
                if (rmw_uros_ping_agent(50, 3) != RMW_RET_OK) {
                    state = AGENT_DISCONNECTED;
                    break;
                }
            }
            (void)rclc_executor_spin_some(&s_executor, RCL_MS_TO_NS(5));
            vTaskDelay(1);
            break;
        case AGENT_DISCONNECTED:
            ESP_LOGW(TAG, "agente perdido: destruyendo entidades");
            destroy_entities();
            state = WAITING_AGENT;
            break;
        }
    }
}

esp_err_t uros_start(void)
{
    init_multiarray(&s_msg_counts, s_counts_buf, NUM_JOINTS, NUM_JOINTS);
    init_multiarray(&s_msg_cmd, s_cmd_buf, CMD_CAPACITY, 0);
    s_msg_limits.data = 0;

    rmw_ret_t r = rmw_uros_set_custom_transport(true, (void *)&s_uart_port, esp32_serial_open,
                                                esp32_serial_close, esp32_serial_write,
                                                esp32_serial_read);
    if (r != RMW_RET_OK) return ESP_FAIL;

    BaseType_t ok = xTaskCreatePinnedToCore(uros_task, "uros", UROS_TASK_STACK, NULL,
                                            UROS_TASK_PRIO, NULL, UROS_TASK_CORE);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}
