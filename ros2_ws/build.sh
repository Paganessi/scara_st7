#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# build.sh — compila ros2_ws en un entorno LIMPIO: solo ROS 2 Jazzy + el agente micro-ROS.
#
# Por qué: colcon "encadena" el workspace a lo que esté cargado al compilar. Si se compila
# con ~/.bashrc cargando otros workspaces, quedan encadenados (y si se carga el agente con
# local_setup.bash en vez de setup.bash, NO queda encadenado y el launch real no lo encuentra).
# Con este script, después basta:  source ros2_ws/install/setup.bash
#
# Uso:  ./build.sh            (argumentos extra van a colcon build, p. ej. --packages-select X)
#       UROS_WS=/otra/ruta ./build.sh
# ---------------------------------------------------------------------------
set -e
cd "$(dirname "$0")"
UROS_WS="${UROS_WS:-$HOME/uros_ws}"
exec env -i HOME="$HOME" USER="${USER:-$(id -un)}" TERM="${TERM:-xterm-256color}" \
    LANG="${LANG:-C.UTF-8}" PATH="/usr/local/bin:/usr/bin:/bin" \
    bash --noprofile --norc -c '
      source /opt/ros/jazzy/setup.bash
      if [ -f "$0/install/setup.bash" ]; then source "$0/install/setup.bash";
      else echo "AVISO: no existe $0/install (sin agente micro-ROS: solo modo simulado)"; fi
      colcon build --symlink-install "$@"' "$UROS_WS" "$@"
