#!/usr/bin/env bash
set -eo pipefail
WS="$HOME/grupo_01_xarm6_ws"
if [ ! -f "$WS/entorno.sh" ]; then
  echo "ERROR: ejecuta primero ./instalar.sh"
  exit 1
fi
source "$WS/entorno.sh"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo
echo "Nodos activos:"
ros2 node list || true
echo
echo "Tópicos principales:"
ros2 topic list | grep -E '^/(joint_states|robot_description|tf|tf_static)$' || true
echo
echo "Una muestra de /joint_states (espera máxima: 5 s):"
timeout 5s ros2 topic echo /joint_states --once || echo "No se recibió /joint_states. Verifica que el robot esté abierto y mueve un slider."
