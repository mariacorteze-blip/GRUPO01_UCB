#!/usr/bin/env bash
set -eo pipefail
WS="$HOME/grupo_01_xarm6_ws"
if [ ! -f "$WS/entorno.sh" ] || [ ! -f "$WS/install/setup.bash" ]; then
  echo "ERROR: el workspace no está instalado. Ejecuta primero ./instalar.sh"
  exit 1
fi
source "$WS/entorno.sh"
ros2 launch grupo01_xarm6_bringup display.launch.py
