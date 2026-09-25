#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# idf.sh — corre idf.py en un entorno LIMPIO, sin ROS 2.
#
# Por qué: ~/.bashrc hace "source /opt/ros/jazzy/setup.bash" en cada terminal, y el
# README de micro_ros_espidf_component exige compilar SIN ROS cargado (si no, el
# build de las librerías de micro-ROS toma paquetes de /opt/ros y falla).
# Este script arranca un bash sin perfil, carga ESP-IDF y le pasa los argumentos a idf.py.
#
# Uso (desde la carpeta del proyecto ESP-IDF):
#   ../idf.sh build                         (firmware/scara_esp32)
#   ../../idf.sh -p /dev/ttyUSB0 flash monitor   (firmware/scara_esp32/test)
# Variables opcionales: IDF_PATH (defecto ~/esp/esp-idf), IDF_PYTHON_ENV_PATH.
# ---------------------------------------------------------------------------
set -e
IDF_DIR="${IDF_PATH:-$HOME/esp/esp-idf}"
EXTRA_ENV=()
[ -n "${IDF_PYTHON_ENV_PATH:-}" ] && EXTRA_ENV+=("IDF_PYTHON_ENV_PATH=$IDF_PYTHON_ENV_PATH")
exec env -i HOME="$HOME" USER="${USER:-$(id -un)}" TERM="${TERM:-xterm-256color}" \
    LANG="${LANG:-C.UTF-8}" PATH="/usr/local/bin:/usr/bin:/bin" "${EXTRA_ENV[@]}" \
    bash --noprofile --norc -c '. "$0/export.sh" > /dev/null && exec idf.py "$@"' "$IDF_DIR" "$@"
