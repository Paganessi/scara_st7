#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# run_clean.sh — corre un comando con ROS 2 + este workspace en un entorno LIMPIO.
#
# Por qué: la terminal integrada de VS Code (instalado como snap) inyecta GTK_PATH,
# LOCPATH, XDG_DATA_DIRS... que apuntan a /snap/code y hacen que RViz, rqt y cualquier
# programa Qt muera con "symbol lookup error: /snap/core20/.../libpthread.so.0".
# Alternativa: usar una terminal normal de Ubuntu (Ctrl+Alt+T) en vez de la de VS Code.
#
# Uso:  ./run_clean.sh ros2 launch scara_bringup scara.launch.py use_fake:=true
#       ./run_clean.sh rqt_graph
# ---------------------------------------------------------------------------
set -e
WS="$(cd "$(dirname "$0")" && pwd)"
exec env -i HOME="$HOME" USER="${USER:-$(id -un)}" TERM="${TERM:-xterm-256color}" \
    LANG="${LANG:-C.UTF-8}" PATH="/usr/local/bin:/usr/bin:/bin" \
    DISPLAY="${DISPLAY:-:0}" XAUTHORITY="${XAUTHORITY:-}" XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" \
    DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-}" WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" \
    ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    bash --noprofile --norc -c 'source "$0/install/setup.bash" && exec "$@"' "$WS" "$@"
