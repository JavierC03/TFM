# Paquete de Manipulación O3DE

Este paquete integra la detección de objetos mediante YOLO con capacidades de manipulación robótica utilizando MoveIt para aplicaciones de robótica en el motor Open 3D Engine (O3DE).

## Descripción

El paquete `manipulation_o3de` proporciona herramientas para la manipulación robótica basada en visión. Permite detectar objetos utilizando el algoritmo YOLO (You Only Look Once) y controlar un brazo robótico mediante MoveIt para interactuar con los objetos detectados.

## Estructura del Paquete

```
manipulation_o3de/
├── CMakeLists.txt    - Configuración de compilación
├── package.xml       - Metadatos del paquete
└── src/             
    ├── test_moveit.cpp - Pruebas básicas de MoveIt
    └── yolo_moveit.cpp - Integración de YOLO con MoveIt
```

## Dependencias

* ROS 2
* MoveIt
* TF2 (Transformaciones)
* YOLO_ROS
* Geometry Messages

## Instalación

1. Asegúrese de tener todas las dependencias instaladas:
   ```bash
   sudo apt install ros-$ROS_DISTRO-moveit
   sudo apt install ros-$ROS_DISTRO-tf2 ros-$ROS_DISTRO-tf2-geometry-msgs
   ```

2. Clone este paquete en su espacio de trabajo:
   ```bash
   cd ~/tfm_ws/src/
   # Si no está ya clonado
   ```

3. Asegúrese de que el paquete yolo_msgs está disponible en su espacio de trabajo.

4. Compile el paquete:
   ```bash
   cd ~/tfm_ws/
   colcon build --packages-select manipulation_o3de
   ```

5. Cargue el entorno:
   ```bash
   source ~/tfm_ws/install/setup.bash
   ```

## Uso

### Prueba de MoveIt

Este ejecutable proporciona una prueba básica de las funcionalidades de MoveIt:

```bash
ros2 run manipulation_o3de test_moveit
```

### Integración YOLO-MoveIt

Este ejecutable integra la detección de objetos mediante YOLO con la planificación de movimiento de MoveIt:

```bash
ros2 run manipulation_o3de yolo_moveit
```

## Funcionalidades

- Detección de objetos mediante el algoritmo YOLO
- Planificación de trayectorias para el brazo robótico utilizando MoveIt
- Transformaciones de coordenadas entre los sistemas de referencia de la cámara y el robot
- Manipulación de objetos basada en la información visual
