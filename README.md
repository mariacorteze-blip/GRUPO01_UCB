IMT-342 ROBÓTICA - Práctica de Cinemática Directa e Inversa (xArm 6)

Grupo 01  
Robot asignado: UFACTORY xArm 6  
Integrantes:
- Ariana Reyes
- Maria Fernanda Cortez

---

## 1. PRIMERA INSTALACIÓN

## Requisitos previos:
- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Conexión a internet estable durante la instalación

### Instrucciones de ejecución:
Abre una terminal dentro del directorio raíz de este repositorio clonado y ejecuta:

```bash
chmod +x instalar.sh abrir.sh recompilar.sh verificar.sh
./instalar.sh

```

El instalador creará automáticamente el workspace principal en la siguiente ruta:
`~/grupo_01_xarm6_ws`

Con la estructura estándar de ROS 2:

```text
~/grupo_01_xarm6_ws/
├── src/
├── build/
├── install/
├── log/
└── entorno.sh

```

> **Nota:** La instalación se realiza una sola vez. El script `instalar.sh` no eliminará un workspace existente para proteger el código que agregues posteriormente.

---

## 2. ABRIR EL ROBOT DESPUÉS DE INSTALAR

### Opción 1: Forma rápida (desde la carpeta del repositorio)

```bash
./abrir.sh

```

### Opción 2: Forma manual (desde la raíz del workspace)

```bash
cd ~/grupo_01_xarm6_ws
source entorno.sh
ros2 launch grupo01_xarm6_bringup display.launch.py

```

### Comprobación visual:

Al ejecutar el comando de visualización deben abrirse:

1. El modelo 3D completo del robot en **RViz2**.
2. La interfaz gráfica **Joint State Publisher GUI**.
3. Los controles deslizantes (*sliders*) para las 6 articulaciones.

*Mueve la articulación 2 (`joint2`) y la articulación 3 (`joint3`) para verificar que el robot cambie de postura adecuadamente.*

---

## 3. TRABAJAR DIRECTAMENTE EN EL WORKSPACE

Cada vez que abras una terminal nueva para trabajar en el proyecto, carga el entorno con:

```bash
cd ~/grupo_01_xarm6_ws
source entorno.sh

```

El script `entorno.sh` realiza dos acciones clave:

* Carga el entorno global de **ROS 2 Jazzy** y el overlay local del workspace.
* Configura `CycloneDDS` como el RMW por defecto (`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`).

Para verificar que el entorno está activo correctamente:

```bash
printenv RMW_IMPLEMENTATION
ros2 pkg list | grep grupo01_xarm6_bringup


## 4. CREACIÓN DEL PAQUETE Y NODOS DE CINEMÁTICA (`grupo01_xarm6_kinematics`)

Sigue estos pasos detallados para crear e integrar el paquete de cinemática directa e inversa según los lineamientos:

### Paso 4.1: Crear el paquete ROS 2 Python

Accede a la carpeta `src` de tu workspace y crea el paquete con sus dependencias:

```bash
cd ~/grupo_01_xarm6_ws/src

ros2 pkg create --build-type ament_python grupo01_xarm6_kinematics --dependencies rclpy sensor_msgs geometry_msgs tf2_ros

```

### Paso 4.2: Copiar los nodos en el paquete

Entra en la carpeta del módulo de Python del paquete recién creado:

```bash
cd ~/grupo_01_xarm6_ws/src/grupo01_xarm6_kinematics/grupo01_xarm6_kinematics

```

Dentro de la carpeta donde se clonó el git, busca la carpeta src/grupo01_xarm6_kinematics/grupo01_xarm6_kinematics, ahí se tienen los siguientes .py

* `fk_node.py` (Nodo de Cinemática Directa)
* `ik_node.py` (Nodo de Cinemática Inversa)

Copia y pegalos en ~/grupo_01_xarm6_ws/src/grupo01_xarm6_kinematics/grupo01_xarm6_kinematics

Otorga permisos de ejecución a los archivos:

```bash
chmod +x fk_node.py ik_node.py
```

### Paso 4.3: Configurar el archivo `setup.py`

Para que ROS 2 reconozca los scripts como ejecutables de consola, debes editar el archivo `setup.py` ubicado en:
`~/grupo_01_xarm6_ws/src/grupo01_xarm6_kinematics/setup.py`

Abre el archivo con tu editor preferido (nano, gedit, VS Code) y modifica la sección `entry_points` para registrar los scripts:


    entry_points={
        'console_scripts': [
            'fk_node = grupo01_xarm6_kinematics.fk_node:main',
            'ik_node = grupo01_xarm6_kinematics.ik_node:main',
        ],
    },


```

### Paso 4.4: Compilación del Workspace

Para compilar el nuevo paquete y actualizar el workspace:

**Opción rápida:**

```bash
./recompilar.sh

```

**Opción manual:**

```bash
cd ~/grupo_01_xarm6_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

```

---

## 5. EJECUCIÓN Y VALIDACIÓN DE LOS NODOS DE CINEMÁTICA

### 5.1 Cinemática Directa (`fk_node`)

1. **Terminal 1:** Lanza el modelo con la interfaz GUI activada:
```bash
cd ~/grupo_01_xarm6_ws
source entorno.sh
ros2 launch grupo01_xarm6_bringup display.launch.py

```


2. **Terminal 2:** Ejecuta el nodo de cinemática directa:
```bash
cd ~/grupo_01_xarm6_ws
source entorno.sh
ros2 run grupo01_xarm6_kinematics fk_node

```


*Al mover los sliders en la GUI, el nodo `fk_node` calculará y mostrará en la terminal las coordenadas $(x, y, z)$ y la orientación (cuaternión) del efector final.*

---

### 5.2 Cinemática Inversa (`ik_node`)

> **¡Importante!** Para ejecutar el nodo de Cinemática Inversa, se debe cerrar o desactivar la interfaz gráfica `joint_state_publisher_gui` para evitar que dos publicadores compitan por el tópico `/joint_states`.

1. **Terminal 1:** Lanza la visualización deshabilitando el GUI manual:
```bash
cd ~/grupo_01_xarm6_ws
source entorno.sh
ros2 launch grupo01_xarm6_bringup display.launch.py use_jsp_gui:=false

```


2. **Terminal 2:** Ejecuta el nodo de cinemática inversa:
```bash
cd ~/grupo_01_xarm6_ws
source entorno.sh
ros2 run grupo01_xarm6_kinematics ik_node

```


3. **Terminal 3:** Envía un punto objetivo al tópico `/target`:
```bash
cd ~/grupo_01_xarm6_ws
source entorno.sh
ros2 topic pub /target geometry_msgs/msg/Point "{x: 0.30, y: 0.10, z: 0.40}" --once

```


*El robot se moverá automáticamente en RViz2 a la posición deseada y el nodo imprimirá en consola la convergencia, el número de iteraciones, el error final y la configuración articular $q^*$.*

---

## 6. COMPROBACIÓN Y DIAGNÓSTICO

Para verificar que el sistema está transmitiendo correctamente los estados articulares:

Con el robot abierto en RViz2, ejecuta desde otra terminal:

```bash
./verificar.sh

```

Debe confirmarse la existencia del tópico `/joint_states`. De lo contrario, asegúrate de haber movido un slider en la interfaz gráfica.

---

## 7. RESUMEN DE TÓPICOS Y HERRAMIENTAS

* **/joint_states** (`sensor_msgs/msg/JointState`): Tópico utilizado por `robot_state_publisher` para actualizar el modelo 3D en RViz2.
* **/target** (`geometry_msgs/msg/Point`): Tópico de entrada para recibir coordenadas cartesianas objetivo $(x_d, y_d, z_d)$ en el nodo de IK.

```

<Elicitations message="¿Deseas verificar algún otro apartado de la documentación o del código?">
  <Elicitation label="Revisar comandos de lanzamiento" query="¿Los comandos del launch file display.launch.py coinciden con la opción use_jsp_gui deshabilitada?"/>
  <Elicitation label="Verificar estructura de paquetes" query="¿La estructura interna de carpetas del workspace cumple con todo lo pedido en la plantilla del informe?"/>
</Elicitations>

```
