# SCARA Estación 7 — Firmware ESP32 y software ROS 2

Robot SCARA RRP (θ1 hombro, θ2 codo, s3 eje Z) para la estación 7 de una línea de ensamble
de PCBs: coloca tapas y paletiza en una matriz 2×2.
Curso **Robótica y Control Digital 2026-2**, Universidad EIA · Grupo 6-F.

| Capa | Tecnología | Carpeta |
|---|---|---|
| Firmware (bajo nivel) | ESP32 · ESP-IDF 5.5 · micro-ROS (Jazzy) | `firmware/scara_esp32/` |
| Software (alto nivel) | ROS 2 Jazzy · Python (rclpy) | `ros2_ws/src/` |

> **Estado actual:** la articulación θ1 tiene motor, polea y encoder. θ2 y Z están completas
> en el software pero deshabilitadas (`joints_enabled` en `scara.yaml`) hasta que se instalen
> sus motores. Todo el sistema se puede ejecutar con un simulador del ESP32.

---

## Arquitectura

```
┌──────────────────────── PC · Ubuntu 24.04 · ROS 2 Jazzy ────────────────────────┐
│  scara_homing ──/scara/raw_cmd───┐                                               │
│  scara_teleop ──/scara/joint_goal┴─> scara_bridge ──/joint_states──> robot_state_ │
│                                      │  (unidades SI, límites,         publisher  │
│                                      │   multiplexor, keepalive)        └─> RViz2  │
│                         /scara/cmd <─┘   /scara/enc_counts, /scara/limits ──┘      │
│                               micro_ros_agent (serial USB, 115200 baud)          │
└───────────────────────────────────────┬──────────────────────────────────────────┘
                                        │ USB (CH9102X)
┌───────────────────────── ESP32 · ESP-IDF + micro-ROS ─────────────────────────────┐
│  PWM (LEDC 20 kHz) → 2× TB6612FNG → 3 motores  ·  PCNT cuadratura x4 ← 3 encoders │
│  PID de posición a 1 kHz · enclavamiento con finales · watchdog · protección bloqueo│
└────────────────────────────────────────────────────────────────────────────────────┘
```

**Criterio de diseño:** en el microcontrolador solo va lo que exige tiempo real duro
(PWM, conteo de encoders, PID a 1 kHz, corte por final de carrera, watchdog). Todo lo demás
se hace en ROS 2: conversión de cuentas a rad/m, límites articulares, homing, generación de
trayectoria y visualización. El ESP32 trabaja únicamente en **cuentas de encoder**, y toda la
calibración física vive en un archivo YAML que se cambia sin reflashear.

### Interfaz ESP32 ↔ ROS 2

| Tópico | Dirección | Tipo | Contenido |
|---|---|---|---|
| `/scara/enc_counts` | ESP32 → ROS (50 Hz) | `std_msgs/Int32MultiArray` | `[c1, c2, c3]` cuentas acumuladas |
| `/scara/limits` | ESP32 → ROS (50 Hz) | `std_msgs/UInt8` | bits 0–5: finales de carrera (1 = presionado) · bit 6: bloqueo · bit 7: watchdog |
| `/scara/cmd` | ROS → ESP32 (≥ 20 Hz) | `std_msgs/Int32MultiArray` | `[modo, v1, v2, v3]` |

| Modo | Valor | Parámetros |
|---|---|---|
| STOP | 0 | — (freno en las tres juntas) |
| POSITION | 1 | metas en cuentas → PID |
| VELOCITY | 2 | duty con signo en ‰ (lazo abierto) |
| ZERO | 3 | `v1` = junta (1–3), `v2` = valor a cargar en el contador |
| SET_GAINS / SET_KD / SET_DUTY | 4 / 5 / 6 | ganancias PID y límites de duty en caliente |

QoS del microcontrolador: *best effort*. `scara_bridge` es el único nodo que publica en `/scara/cmd`.

---

## Estructura del repositorio

```
firmware/
├── idf.sh                    ejecuta idf.py en un entorno sin ROS 2
└── scara_esp32/              proyecto ESP-IDF (firmware con micro-ROS)
    ├── main/                 pines, motores, encoders, finales, control 1 kHz, nodo micro-ROS
    │   └── ctrl_logic.c/.h   lógica pura del control (sin dependencias de hardware)
    ├── test/                 firmware de prueba de hardware (consola serie, sin micro-ROS)
    └── host_test/            pruebas unitarias de la lógica en el PC (gcc)
ros2_ws/
├── build.sh                  compila el workspace en un entorno limpio
├── run_clean.sh              ejecuta un comando (RViz, rqt) en un entorno limpio
└── src/
    ├── scara_bridge/         puente ROS ↔ ESP32: conversiones, límites, multiplexor, /joint_states
    ├── scara_homing/         rutina de homing (servicio /scara/home)
    ├── scara_teleop/         goto (punto a punto) y jog (teclado)
    ├── scara_description/    modelo URDF/xacro
    ├── scara_bringup/        launch del sistema y parámetros (config/scara.yaml)
    ├── scara_tools/          simulador, calibración, sintonía, repetibilidad, grafo ROS
    └── scara_fw_sim/         compila la lógica del firmware para el simulador
```

---

## Requisitos

- Ubuntu 24.04 con **ROS 2 Jazzy** (`ros-jazzy-desktop`) y además:
  ```bash
  sudo apt install ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui ros-jazzy-rqt-graph \
      python3-vcstool python3-serial graphviz
  sudo usermod -aG dialout $USER        # acceso al puerto serie (cerrar sesión después)
  ```
- **ESP-IDF v5.5.5** en `~/esp/esp-idf`:
  ```bash
  sudo apt install git wget flex bison gperf python3-pip python3-venv cmake ninja-build ccache \
      libffi-dev libssl-dev dfu-util libusb-1.0-0
  mkdir -p ~/esp && cd ~/esp
  git clone -b v5.5.5 --recursive --depth 1 --shallow-submodules https://github.com/espressif/esp-idf.git
  cd esp-idf && ./install.sh esp32
  . ./export.sh && pip install catkin_pkg colcon-common-extensions lark empy==3.3.4
  ```
- **Agente micro-ROS** en un workspace aparte (`~/uros_ws`):
  ```bash
  mkdir -p ~/uros_ws/src && cd ~/uros_ws
  git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
  env -i HOME=$HOME PATH=/usr/bin:/bin bash --noprofile --norc -c '
    source /opt/ros/jazzy/setup.bash && cd ~/uros_ws && colcon build &&
    source install/setup.bash && (ros2 run micro_ros_setup create_agent_ws.sh || true) &&
    colcon build'
  ```

> **Notas de entorno.** Si `~/.bashrc` carga ROS 2, compila el firmware con `firmware/idf.sh`,
> porque micro-ROS no compila con ROS cargado. La terminal integrada de VS Code instalado como
> *snap* impide abrir RViz y rqt; usa una terminal normal o `ros2_ws/run_clean.sh`.

---

## Compilación

```bash
git clone https://github.com/Paganessi/scara_st7.git ~/scara_st7

# Software ROS 2
cd ~/scara_st7/ros2_ws && ./build.sh
source install/setup.bash

# Firmware (la primera compilación descarga y compila micro-ROS, ~5 min)
cd ~/scara_st7/firmware/scara_esp32 && ../idf.sh build
cd ~/scara_st7/firmware/scara_esp32/test && ../../idf.sh build
```

## Pruebas

```bash
cd ~/scara_st7/firmware/scara_esp32/host_test && make test     # lógica del firmware (C)
cd ~/scara_st7/ros2_ws && colcon test && colcon test-result --all   # linters, pytest y pruebas C
```

---

## Uso

### Flashear el ESP32
El puerto aparece como `/dev/ttyUSB0` o `/dev/ttyACM0`.
```bash
# Firmware de prueba de hardware (consola interactiva por serie)
cd ~/scara_st7/firmware/scara_esp32/test && ../../idf.sh -p /dev/ttyUSB0 flash monitor
#   m <j> <duty‰> <ms>  pulso · s  stop · c <j>  rueda libre · z <j>  cero · h  ayuda

# Firmware del robot (micro-ROS por UART0; no tiene consola)
cd ~/scara_st7/firmware/scara_esp32 && ../idf.sh -p /dev/ttyUSB0 flash
```

### Lanzar el sistema
```bash
source ~/scara_st7/ros2_ws/install/setup.bash
ros2 launch scara_bringup scara.launch.py use_fake:=false serial_port:=/dev/ttyUSB0   # robot real
ros2 launch scara_bringup scara.launch.py use_fake:=true                              # simulador
# opciones: rviz:=false · require_homed:=false · config:=/ruta/a/otro.yaml
```

### Operación
```bash
ros2 service call /scara/home std_srvs/srv/Trigger       # homing
ros2 topic echo /scara/homing_state
ros2 run scara_teleop goto 30 0 0                         # θ1 [°], θ2 [°], s3 [mm]
ros2 run scara_teleop jog                                 # movimiento con el teclado
ros2 service call /scara/stop std_srvs/srv/Trigger        # detener
```

### Puesta en marcha y calibración
```bash
ros2 run scara_tools calibrate_joint 1 --angle 90 --serial /dev/ttyUSB0   # cuentas por radián
ros2 run scara_tools deadzone_test 1                                     # duty mínimo de arranque
ros2 run scara_tools step_test 1 300 --kp 2000 --kd 10 --duty-min 72     # respuesta al escalón
ros2 run scara_tools repeat_home -n 5                                    # repetibilidad del homing
ros2 run scara_tools graph_snapshot                                      # grafo ROS (como rqt_graph)
```
`deadzone_test` y `step_test` se ejecutan con el bridge apagado.

---

## Configuración

Todo lo calibrable está en `ros2_ws/src/scara_bringup/config/scara.yaml`:

| Grupo | Parámetros |
|---|---|
| Transmisión | `counts_per_motor_rev`, `gear_ratio`, `transmission_ratio`, `lead_mm_per_rev`, `joint_sign`, `counts_per_rad` / `counts_per_m` (calibración medida) |
| Juntas | `joints_enabled`, `lower_limits`, `upper_limits` |
| Control | `pid_kp_milli`, `pid_ki_milli`, `pid_kd_milli`, `duty_min`, `duty_max`, `max_velocity` |
| Homing | `homing_order`, `home_switch`, `home_offset`, `homing_fast`, `homing_slow`, `backoff_counts`, `homing_timeout_s`, `rest_pose` |

Los pines del ESP32 están en `firmware/scara_esp32/main/pins.h`, y los signos de motor y
encoder en `firmware/scara_esp32/main/config.h`.

### Geometría (Denavit–Hartenberg, convención Tsai / Crane–Duffy)

| Junta | Tipo | a (mm) | α (°) | S (mm) | θ |
|---|---|---|---|---|---|
| 1 | R | 0 | 0 | 178 | θ1 |
| 2 | R | 160 | 0 | 0 | θ2 |
| 3 | P | 130 | 180 | s3 | 0 |

Cinemática directa: `x = r1·cos θ1 + r2·cos(θ1+θ2)`, `y = r1·sin θ1 + r2·sin(θ1+θ2)`,
`z = h − s3`. Alcance máximo 290 mm. Límites: θ1 ±90°, θ2 ±140°.

---

## Hardware

- ESP32 DevKit de 30 pines (ESP-WROOM-32, USB CH9102X).
- 2× TB6612FNG y 3× Pololu 37D 12 V con encoder de 64 CPR.
- 6 finales de carrera normalmente abiertos (activos en bajo).
- Encoders a 12 V con divisor 3.3 kΩ / 1.2 kΩ hacia el ESP32.

## Referencias

- micro-ROS, [micro_ros_espidf_component](https://github.com/micro-ROS/micro_ros_espidf_component) (rama jazzy): el transporte serial se basa en el ejemplo `int32_publisher_custom_transport` (Apache-2.0).
- Espressif, [ESP-IDF Programming Guide v5.5](https://docs.espressif.com/projects/esp-idf/en/v5.5/): PCNT (ejemplo `rotary_encoder`), LEDC, FreeRTOS.
- Toshiba, *TB6612FNG Datasheet*. Pololu, *37D Metal Gearmotors with 64 CPR Encoder*.
- ROS 2, [rqt_graph](https://github.com/ros-visualization/rqt_graph): generador de grafos usado por `graph_snapshot`.
- K. J. Åström y T. Hägglund, *PID Controllers: Theory, Design, and Tuning*.
- [REP 103](https://www.ros.org/reps/rep-0103.html): unidades y convenciones de coordenadas.
