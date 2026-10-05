#!/usr/bin/env bash
set -eo pipefail
WS="$HOME/grupo_01_xarm6_ws"
if [ ! -d "$WS/src" ]; then
  echo "ERROR: no existe $WS/src. Ejecuta primero ./instalar.sh"
  exit 1
fi
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd "$WS"
colcon build --symlink-install
source "$WS/install/setup.bash"
echo "Workspace recompilado correctamente: $WS"
