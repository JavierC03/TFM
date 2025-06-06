# Mi Paquete de Navegación y SLAM

Este paquete integra la pila de Navegación 2 (Nav2) con capacidades de Localización y Mapeo Simultáneos (SLAM) para robots móviles. Proporciona una solución completa para la navegación autónoma en entornos desconocidos.

## Estructura del Paquete

- **config/**: Contiene archivos de configuración para Nav2 y SLAM.
  - `nav2_params.yaml`: Parámetros para la pila Nav2.
  - `slam_params.yaml`: Parámetros para el algoritmo SLAM.
  - `rviz_config.rviz`: Configuración de visualización para RViz.

- **launch/**: Contiene archivos de lanzamiento para iniciar los sistemas de navegación y SLAM.
  - `nav2_launch.py`: Lanza la pila Nav2.
  - `slam_launch.py`: Lanza los nodos SLAM.
  - `navigation_with_slam.launch.py`: Combina los lanzamientos de Nav2 y SLAM.

- **maps/**: Directorio para almacenar mapas generados.

- **CMakeLists.txt**: Configuración de compilación para el paquete.

- **package.xml**: Metadatos sobre el paquete.

## Instrucciones de Configuración

1. **Instalar Dependencias**: Asegúrese de que todas las dependencias requeridas para Nav2 y SLAM estén instaladas en su entorno ROS.

2. **Compilar el Paquete**: Navegue al directorio del workspace y ejecute:
   ```
   colcon build --packages-select my_nav2_slam_pkg
   ```

3. **Cargar el Archivo de Configuración**: Después de compilar, cargue el archivo de configuración:
   ```
   source install/setup.bash
   ```

## Uso

Para lanzar la pila de navegación con capacidades SLAM, use el siguiente comando:
```
ros2 launch my_nav2_slam_pkg navigation_with_slam.launch.py
```

Esto iniciará los nodos necesarios para navegación y SLAM, permitiendo que su robot navegue y mapee su entorno simultáneamente.
