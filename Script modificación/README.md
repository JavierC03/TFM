# Script de Modificación de Prefabs

Este script permite generar variaciones aleatorias de archivos prefab modificando las posiciones de objetos. Está diseñado específicamente para alterar coordenadas X e Y de elementos en un prefab, permitiendo crear múltiples variaciones para simulaciones o entrenamiento.

## Características

- Copia un archivo prefab original a un directorio destino
- Intercambia posiciones entre objetos pequeños dentro de los objetos TOY
- Modifica aleatoriamente las posiciones X e Y de cubos dentro de rangos especificados
- Limpia automáticamente el directorio destino antes de generar nuevos prefabs

## Requisitos

- Python 3.x
- Módulos estándar: shutil, os, random, re, argparse

## Uso

```bash
python crear_prefab.py --origen <ruta_archivo_origen.prefab> --destino <carpeta_destino>
```

### Argumentos

- `--origen`: Ruta completa al archivo .prefab original que se va a modificar
- `--destino`: Carpeta donde se guardará el archivo prefab modificado (RandomPositions.prefab)

## Funcionamiento

El script realiza las siguientes operaciones:

1. Limpia el directorio destino (si existe)
2. Copia el archivo .prefab original al directorio destino
3. Intercambia posiciones entre objetos pequeños dentro de TOY
4. Modifica aleatoriamente las posiciones X e Y de los cubos especificados
5. Guarda el archivo modificado como "RandomPositions.prefab"

## Configuración

Las líneas y rangos de modificación están predefinidos en el script:

- **Objetos a modificar**: Se especifican mediante números de línea en el archivo prefab
  - Cubos principales: líneas 2223, 2828, 3433, 4038, 4643, 5248 (X) y 2224, 2829, 3434, 4039, 4644, 5249 (Y)
  - Objetos TOY: líneas 1257, 1375, 1919 (X) y 1258, 1376, 1920 (Y)

- **Rangos de posición aleatoria**:
  - X: entre 1.456009 y 2.2069075
  - Y: entre 1.9625132 y 3.4060755

## Ejemplo

```bash
python crear_prefab.py --origen /ruta/al/original.prefab --destino /carpeta/salida/
```

Este comando generará un nuevo archivo `/carpeta/salida/RandomPositions.prefab` con posiciones modificadas aleatoriamente.

## Notas

- El script asume que las líneas especificadas contienen valores de posición en un formato específico
- Se recomienda hacer una copia de seguridad de los archivos originales antes de ejecutar el script