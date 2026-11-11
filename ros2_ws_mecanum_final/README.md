# Examen final Robótica Móvil: Robot Mecanum con LiDAR y planeación A*

| | |
|---|---|
| Duración | **40 minutos** |
| Valor | **2.5 Pts** |
| Herramientas | ROS 2 Jazzy + Gazebo Harmonic, Python |

El robot Mecanum del examen anterior **está incompleto**: le falta la **rueda trasera derecha** y el **RPLIDAR A3**. Debe completarlo, programar su cinemática (**Q, W, J**) y terminar el planificador **A\*** para que el robot llegue a la meta que le asigna su carné, esquivando los obstáculos de la pista.

## Cómo empezar

```bash
cd ~
git clone <este repositorio> ros2_ws_mecanum_final
cd ~/ros2_ws_mecanum_final
colcon build --symlink-install && source install/setup.bash

# 1. Vea SUS datos (geometría, LiDAR y meta) y pruebe su código sin Gazebo
ros2 run mi_mecanum_control probar_offline <su_carne>
```

Todos los valores salen de los **últimos 4 dígitos de su carné**. Cada estudiante tiene un robot y una meta distintos: trabaje solo con los suyos.

## Hoja de datos

- [RPLIDAR A3 — Especificaciones (Slamtec)](https://www.slamtec.com/en/lidar/a3spec)

Dimensiones, masa, muestras por segundo y rango de cada modo deben salir de la hoja de datos.

## Parte 1 — Robot (0.6 Pts, ~10 min) · `mi_robot_mecanum/urdf/mi_robot.urdf.xacro`

1. **(R-1)** Instanciar la rueda trasera derecha `tra_der` con la macro `rueda` (posición con lx = wheelbase/2, ly = track/2 y los signos correctos) y agregar su **motor** con la macro `motor`.
2. **(R-2)** Agregar el **RPLIDAR A3**: link `lidar_link` (cilindro con dimensiones, masa e inercia reales), joint fijo `joint_lidar` en la posición (x, 0, z) de su carné, y el sensor `gpu_lidar` en `/scan` con `gz_frame_id` = `lidar_link`, `update_rate` = barrido, `samples` = muestras por segundo ÷ barrido, ángulo de −π a π, rango del modo y ruido gaussiano σ = 0.01 m.

## Parte 2 — Cinemática (0.7 Pts, ~10 min) · `mi_mecanum_control/cinematica.py`

Con ξ = [u, v, r]ᵀ, ω = [ω_di, ω_dd, ω_ti, ω_td]ᵀ, a = radio de rueda y k = lx + ly:

1. **(C-1)** Matriz **Q** (4×3) de la cinemática inversa **ω = Qξ** (las ecuaciones de cada rueda están en el encabezado del archivo).
2. **(C-2)** Matriz **W** (3×4) de la cinemática directa **ξ = Wω**, con W = (QᵀQ)⁻¹Qᵀ.
3. **(C-3)** Matriz **J(φ)** (3×3) de la odometría **η̇ = J(φ)ξ**.

`probar_offline` debe mostrar **W·Q = I ? SÍ**.

## Parte 3 — Planeación A\* (0.8 Pts, ~12 min) · `mi_mecanum_control/astar.py`

La grilla del mundo (`mapa.py`) ya viene construida: celdas de 0.1 m, obstáculos inflados con el radio del robot y 8 vecinos (costo 1 recto, √2 en diagonal). Complete:

1. **(A\*-1)** `heuristica(a, b)`: distancia euclidiana entre celdas.
2. **(A\*-2)** Dentro del ciclo: costo g del vecino pasando por la celda actual; si mejora, guardar `g`, `vino_de`, calcular **f = g + h** y agregarlo a la cola.
3. **(A\*-3)** `reconstruir_camino`: recorrer `vino_de` desde la meta hasta el inicio y devolver el camino en orden.

`probar_offline` debe mostrar el **costo de A\*** y llegar a la meta en la simulación ideal.

## Parte 4 — Ejecución en Gazebo (0.4 Pts, ~5 min)

```bash
# Terminal 1
ros2 launch mi_robot_mecanum simulacion.launch.py carne:=<su_carne>
# Terminal 2 (ejecútelo dentro de ~/ros2_ws_mecanum_final para que el CSV quede ahí)
cd ~/ros2_ws_mecanum_final && source install/setup.bash
ros2 run mi_mecanum_control navegar_astar --ros-args -p carne:=<su_carne> -p use_sim_time:=true
```

El robot debe llegar a la meta **sin chocar**. En RViz deben verse el robot, el LiDAR (`/scan`), el mapa de A\* (`/mapa`) y el camino (`/plan`), con el escaneo alineado con los obstáculos del mapa. El nodo guarda `resultado_<carne>.csv`.

## Entrega (~3 min)

Suba a Moodle **un solo archivo** `RM_NNAA.zip` (NN = nombre, AA = apellido) con:

1. Su workspace `ros2_ws_mecanum_final` **sin** las carpetas `build/`, `install/` ni `log/`.
2. El archivo `resultado_<carne>.csv` que genera el nodo `navegar_astar`.
3. Una captura `captura_<carne>.png` de pantalla completa donde se vean Gazebo y RViz con el robot en la meta, la terminal con el mensaje **META ALCANZADA** y la sesión activa de Moodle con el examen y su usuario.

Ponga el CSV y la captura dentro de `~/ros2_ws_mecanum_final` y comprima así:

```bash
cd ~
zip -r RM_NNAA.zip ros2_ws_mecanum_final -x "*/build/*" "*/install/*" "*/log/*"
```

## Rúbrica

| Criterio | Peso |
|---|---|
| Rueda `tra_der` con su motor y RPLIDAR A3 según hoja de datos y carné | 0.6 |
| Q, W y J(φ) correctas (W·Q = I) | 0.7 |
| A\*: heurística, costo g y reconstrucción del camino (costo óptimo) | 0.8 |
| Robot llega a la meta en Gazebo sin chocar (CSV y captura) | 0.4 |
