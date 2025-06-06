# Espacio de Trabajo 

Este espacio de trabajo contiene diferentes paquetes ROS 2 y herramientas para simulación robótica, navegación autónoma, SLAM y manipulación de objetos utilizando visión artificial.

## Estructura del Espacio de Trabajo

### basic_nav
Paquete que proporciona funcionalidades básicas de navegación para robots utilizando ROS 2 y Nav2. Permite la ejecución de navegación autónoma siguiendo puntos de referencia predefinidos (waypoints).

### Guia Instalación O3DE
Documentación detallada sobre el proceso de instalación, configuración y primeros pasos con Open 3D Engine (O3DE) en entorno Linux, así como su integración con ROS 2 Robotic Manipulation Template para la simulación de brazos robóticos.

### manipulation_o3de
Paquete que integra la detección de objetos mediante YOLO con capacidades de manipulación robótica utilizando MoveIt para aplicaciones de robótica en el motor Open 3D Engine (O3DE).

### my_nav2_slam_pkg
Paquete que integra la pila de Navegación 2 (Nav2) con capacidades de Localización y Mapeo Simultáneos (SLAM) para robots móviles, proporcionando una solución completa para la navegación autónoma en entornos desconocidos.

### Script modificación
Script para generar variaciones aleatorias de archivos prefab modificando las posiciones de objetos. Diseñado específicamente para alterar coordenadas X e Y de elementos en prefabs de O3DE, permitiendo crear múltiples variaciones para simulaciones o entrenamiento.

## Requisitos del Sistema

- ROS 2 (preferiblemente Humble)
- Nav2
- MoveIt
- Open 3D Engine (O3DE)
- Python 3.x
- Dependencias específicas de cada paquete

## Instalación

Cada paquete tiene sus propios requisitos e instrucciones de instalación. Consulte los archivos README.md individuales dentro de cada carpeta para obtener información detallada.

## Uso

Este workspace está diseñado para facilitar el desarrollo y la investigación en robótica autónoma, combinando navegación, SLAM, visión por computadora y manipulación robótica en diferentes entornos de simulación.

