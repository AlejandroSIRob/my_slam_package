# Proyecto de SLAM con Algoritmo de Split and Merge

Este proyecto contiene un sistema completo de SLAM (Simultaneous Localization and Mapping), que ha sido diseñado y probado específicamente para funcionar dentro del entorno de simulación Gazebo, utilizando ROS Kinetic como middleware de comunicación entre nodos, y con una implementación personalizada del algoritmo Split and Merge. El objetivo principal de este sistema es detectar segmentos en el entorno mediante datos LIDAR, extraer landmarks relevantes y realizar estimaciones de posición utilizando un filtro EKF.

---

## 📁 Estructura del paquete `my_slam_package`

El paquete `my_slam_package` se encuentra dentro del espacio de trabajo `catkin_ws`, y presenta la siguiente estructura de carpetas:

```
catkin_ws/
└── src/
    └── my_slam_package/
        ├── CMakeLists.txt
        ├── config/
        │   └── custom_slam.rviz
        ├── launch/
        │   ├── slam_sim.launch
        │   ├── slam_sim_2.launch
        │   ├── slam_sim_3.launch
        │   ├── slam_sim_4.launch
        │   └── split.launch
        ├── package.xml
        ├── README.md
        ├── scripts/
        │   ├── ekf_slam.py
        │   └── split.py
        └── src/
```

- El archivo `split.py`, ubicado dentro de `scripts/`, contiene la implementación del algoritmo de segmentación por divisiones recursivas.
- El archivo `split.launch` es el encargado de lanzar el nodo de segmentación junto con el entorno de simulación y visualización en `rviz`.

---

## ▶️ Ejecución del sistema

La ejecución se realiza desde el entorno de terminal de ROS en el directorio raíz del espacio de trabajo (`catkin_ws`), mediante el siguiente comando:

```bash
roslaunch my_slam_package split.launch
```

---

## ⚙️ Requisitos y dependencias

### Opción 1: Máquina virtual preconfigurada

Para facilitar la ejecución y evitar problemas de compatibilidad, se recomienda usar la siguiente máquina virtual, ya configurada y probada:

- **Descarga directa**: [Máquina Virtual SLAM ROS](https://nextcloud.citius.usc.es/s/2oJ8AXpRxKmzKLR)
- **Contraseña**: `robotica`

Esta opción incluye todas las dependencias necesarias preinstaladas, por lo que se puede lanzar el sistema directamente desde un terminal en la VM.

---

### Opción 2: Instalación manual en sistema Ubuntu

#### Versión recomendada de sistema operativo:

- Ubuntu 16.04 (ROS Kinetic)
- Ubuntu 18.04 (ROS Melodic)

#### Dependencias de ROS necesarias (declaradas en `package.xml`):

```xml
<build_depend>geometry_msgs</build_depend>
<build_depend>nav_msgs</build_depend>
<build_depend>rospy</build_depend>
<build_depend>sensor_msgs</build_depend>
<build_depend>std_msgs</build_depend>
<build_depend>tf</build_depend>
<build_depend>visualization_msgs</build_depend>
```

#### Librerías Python requeridas:

```python
import rospy
import numpy as np
import random  
import json
import tf
```

#### Requisitos adicionales:

- **Python 2.7.12**, instalado por defecto en sistemas Ubuntu compatibles con ROS.
- **Gazebo**, incluido con la instalación de ROS, para simular el entorno y el robot.

---


## 📌 Notas finales

Este paquete forma parte de una práctica avanzada de percepción robótica en simulación; permite comprender el flujo completo desde la adquisición de datos con LIDAR hasta la extracción de segmentos y posterior estimación de la pose del robot mediante SLAM. Se recomienda analizar con detalle el código de `split.py` y utilizar `rviz` para la visualización clara de los segmentos generados, lo cual es esencial para el entendimiento y depuración del sistema.
