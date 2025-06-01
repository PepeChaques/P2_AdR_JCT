# Práctica 2: Filtro de Kalman en ROS 2
Este repositorio contiene el código base para la **Práctica 2** de la asignatura de *Ampliación de Robótica*, cuyo objetivo es implementar un **Filtro de Kalman (KF)** en un entorno simulado con **ROS 2**.

El ejercicio se divide en dos partes: una primera aproximación basada en odometría con estimación de posición, y una segunda con estimación de posición y velocidad utilizando un modelo de estado extendido.

---

## Estructura del repositorio
 - kalman_filter.py # Implementación del KF con TODOs para completar 
 - kf_estimation.py # Nodo con el modelo básico de KF (posición)
 - kf_estimation_vel.py # Nodo con el modelo completo de KF (posición y velocidad) 
 - motion_models.py # Modelos de movimiento A y B 
 - observation_models.py # Modelos de observación C
 - sensor_utils.py # Funciones de alto nivel para facilitar las cosas con los sensores
 - visualization.py # Funciones de visualización de resultados
 

## Instrucciones

### Requisitos previos
Descargar el simulador y los paquetes dependientes del mismo para poder trabajar con el robot Turtlebot 4:

```bash
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-turtlebot4-simulator ros-humble-irobot-create-nodes ros-dev-tools

```

### 1. Clonar el repositorio
Fuera del docker (en tu ubuntu o en el WSL)

```bash
mkdir -p ~/AdR/p2_ws/src
cd p2_ws/src
git clone https://github.com/miggilcas/p2_kf_adr
cd p2_kf_adr
```
### 2. Construir el paquete
Ya dentro del Docker:
```bash
cd ~/AdR/p2_ws
colcon build --packages-select p2_kf_adr
source install/setup.zsh  # o setup.bash si no estás usando el docker
```
### 3. Lanzar el simulador
```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true
```
### 4. Ejecutar el nodo del filtro de Kalman
#### Modelo 1: estimación de posición
```bash
ros2 run p2_kf_adr kf_estimation
```
#### Modelo 2:
```bash
ros2 run p2_kf_adr kf_estimation_vel
```

## Objetivo de la práctica

- Comprender y programar un filtro de Kalman básico para estimar la posición del robot.
- Ampliar el modelo de estado para incluir velocidad y emplear un modelo lineal puro.
- Comparar el comportamiento del filtro con diferentes configuraciones de ruido.
- Preparar el terreno para el uso de un Filtro de Kalman Extendido (EKF) en la siguiente práctica.

## Resultados de los experimentos
Los archivos kalman_filter.py, motion_models.py y observation_models.py han sido modificados para implementar filtros de kalman.

Lamentablemente los experimentos no se han podido realizar debido a que el uso de wsl no es capaz de soportar el programa del turtlebot4, sin embargo al runear los ficheros kf_estimation y kf_estimation_vel las gráficas salen, por lo que asumo que los programas funcionan de forma correcta.

Sin embargo el programa está creado para facilitar la posible implementación de los experimentos, teniendo el ruido incorporado valores bajos por defecto. 
Si se quiere introducir valores de ruido altos solo en la medida se deberá multiplicar x10 los valores de ruido de la variable **obs_noise_std**, disponibles en el archivo **kalman_filter.py**, de la carpeta **filters**, tanto en la class **KalmanFilter** como en la **KalmanFilter_2**, mientras que para generar valores de ruido altos solo en la predicción habría que multiplicar x10 los valores de ruido de la variable **proc_noise_std**, los cuales están en el mismo archivo y las mismas clases que la variable anterior.

## Estructura de los filtros

Los filtros se estructuran en tres partes fundamentales.

- Init

En esta parte se introduce todo lo necesario para que funcione el filtro de kalman, iniciando las variables e introduciendo los modelos matemáticos utilizados en el resto del programa.

- Predict

 Esta parte se encarga de generar una predicción del estado del robot antes de recibir una medición de los sensores, esto sucede cuando una acción como el movimiento del robot se realiza.

- Update

En esta parte se realiza una actualización del estado cuando se recibe una medida externa (generalmente de un sensor), esto permite que el propio filtro se corrija a sí mismo, mezclando los datos del Predict y el Update para llegar al resultado final.
