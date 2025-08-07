# Yasmin Waypoint Navigation

Este paquete implementa navegación por waypoints utilizando una máquina de estados YASMIN para ROS2.

## Descripción

Reimplementación del script `waypoints_navigation.py` usando YASMIN para una navegación más robusta y estructurada.

## Estados de la Máquina

1. **INIT_NAVIGATION**: Inicializa Nav2 y carga waypoints
2. **LOAD_NEXT_WAYPOINT**: Carga el siguiente waypoint
3. **NAVIGATE_TO_WAYPOINT**: Navega al waypoint actual
4. **WAYPOINT_REACHED**: Procesa la llegada al waypoint
5. **UPDATE_WAYPOINT_INDEX**: Actualiza el índice
6. **FINISH**: Todos los waypoints completados
7. **ERROR**: Manejo de errores

## Uso

1. Compila el paquete:
```bash
cd /home/user/tfm_ws
colcon build --packages-select yasmin_waypoint_nav
source install/setup.bash
```

2. Ejecuta:
```bash
ros2 run yasmin_waypoint_nav yasmin_waypoint_navigator
```

## Configuración

Edita `config/waypoints.yaml` para definir tus waypoints:

```yaml
- x: 4.43
  y: -11.7
  z: 0.0
  w: 0.0
```

## Dependencias

- ROS2 Humble
- Nav2
- YASMIN
- nav2_simple_commander
