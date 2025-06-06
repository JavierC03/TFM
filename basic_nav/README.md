# Paquete Basic Nav

Este paquete proporciona funcionalidades básicas de navegación para robots utilizando ROS 2 y Nav2. Permite la ejecución de navegación autónoma siguiendo puntos de referencia predefinidos (waypoints).

## Estructura del Paquete

- **basic_nav/**: Contiene los scripts principales de Python
  - `simple_commander_example.py`: Ejemplo básico del uso de Nav2 Simple Commander
  - `waypoints_navigation.py`: Script para navegación autónoma por waypoints

- **config/**: Archivos de configuración
  - `waypoints.yaml`: Definición de los puntos de referencia para la navegación

- **resource/**: Recursos del paquete

- **test/**: Pruebas automatizadas para el código

## Requisitos

- ROS 2
- Nav2
- Paquetes Python: rclpy, nav_msgs, geometry_msgs

## Instalación

1. Clone este paquete en su espacio de trabajo de ROS 2:
   ```
   cd ~/tfm_ws/src/
   ```

2. Compile el paquete:
   ```
   cd ~/tfm_ws/
   colcon build --packages-select basic_nav
   ```

3. Cargue el entorno:
   ```
   source ~/tfm_ws/install/setup.bash
   ```

## Uso

### Navegación por Waypoints

Este nodo permite que el robot navegue automáticamente a través de una serie de puntos definidos en un archivo YAML.

1. Asegúrese de que su robot esté correctamente configurado con Nav2.

2. Opcionalmente, edite el archivo de waypoints en `config/waypoints.yaml` para definir sus propios puntos.

3. Ejecute el nodo de navegación:
   ```
   ros2 run basic_nav waypoints_navigation
   ```

### Ejemplo Simple Commander

Este nodo proporciona un ejemplo básico de cómo usar el Nav2 Simple Commander:

```
ros2 run basic_nav simple_commander_example
```

## Configuración de Waypoints

Los waypoints se definen en el archivo `config/waypoints.yaml` con el siguiente formato:

```yaml
waypoints:
  - x: 1.0    # Posición x en metros
    y: 1.0    # Posición y en metros
    z: 0.0    # Orientación z (componente del cuaternio)
    w: 1.0    # Orientación w (componente del cuaternio)
  - x: 2.0
    y: 2.0
    z: 0.0
    w: 1.0
```

