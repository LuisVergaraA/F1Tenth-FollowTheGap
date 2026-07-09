# 🏎️ F1Tenth — Controlador Reactivo (Follow the Gap)

Este repositorio contiene la implementación individual de un paquete de ROS 2 (Humble) con un controlador reactivo basado en el algoritmo **Follow the Gap** para el simulador oficial de F1Tenth. El proyecto está dividido en dos partes:

- **Parte 1:** el vehículo principal completa **10 vueltas consecutivas sin colisiones** en la pista de **BrandsHatch** limpia (sin obstáculos), en el menor tiempo posible.
- **Parte 2:** sobre la misma pista, ahora con **5 obstáculos estáticos** repartidos en el trazado, y con un **segundo vehículo (oponente)** controlado por su propio nodo Follow the Gap, actuando como obstáculo dinámico.

> Este README asume una computadora **nueva**, sin nada instalado salvo Ubuntu 22.04. Cada paso está explicado para que no dependa de configuración previa en la máquina del evaluador.

---

## ✅ 0. Prerrequisitos

- **Ubuntu 22.04** con **ROS 2 Humble** instalado (`sudo apt install ros-humble-desktop`).
- Dependencias de mensajes usadas por el simulador y los nodos:
  ```bash
  sudo apt install ros-humble-ackermann-msgs ros-humble-nav-msgs python3-colcon-common-extensions
  pip3 install numpy --break-system-packages
  ```
- El **simulador oficial de F1Tenth** instalado de forma nativa (sin Docker), siguiendo la guía de:
  👉 [widegonz/F1Tenth-Repository](https://github.com/widegonz/F1Tenth-Repository)

  Verifica que compila y corre el ejemplo por defecto **antes** de seguir con este README:
  ```bash
  cd ~/F1Tenth-Repository
  colcon build
  source install/setup.bash
  ros2 launch f1tenth_gym_ros gym_bridge_launch.py
  ```
  Si se abre RViz con la pista de Levine, el simulador está listo.

Este repositorio **no incluye el simulador en sí** — es un controlador (más los mapas de la pista) que se ejecuta *sobre* el simulador ya instalado, comunicándose por tópicos de ROS 2 (`/scan`, `/drive`, `/ego_racecar/odom`, `/opp_scan`, `/opp_drive`).

---

## 📦 1. Estructura de este repositorio

Un único paquete ROS 2 (`ament_python`) contiene **ambos nodos** (auto principal y auto oponente) y **ambos mapas** (limpio y con obstáculos):

```
F1Tenth-FollowTheGap/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── reactive_gap_follow
├── test/
│   └── ...
├── maps/
│   ├── BrandsHatch_map.png / .yaml             ← Parte 1: pista limpia
│   └── BrandsHatch_obstacles_map.png / .yaml   ← Parte 2: pista + 5 obstáculos
└── reactive_gap_follow/
    ├── __init__.py
    ├── reactive_gap_follow_node.py    ← auto principal (ego): Follow the Gap + evasión de obstáculos
    └── dynamic_obstacles_node.py      ← auto oponente (opp): mismo algoritmo, más lento
```

Todo lo necesario para correr **ambas partes** del proyecto está en este único repositorio — no hace falta descargar nada más aparte del simulador base.

---

## 🛠️ 2. Instalación y compilación

1. Dirígete a la carpeta `src` de tu workspace del simulador F1Tenth (por ejemplo, `~/F1Tenth-Repository/src`) y clona el repositorio:

   ```bash
   cd ~/F1Tenth-Repository/src
   git clone https://github.com/LuisVergaraA/F1Tenth-FollowTheGap.git reactive_gap_follow
   ```

   > ⚠️ Nota el `reactive_gap_follow` al final del `git clone` — esto asegura que la carpeta del paquete se llame igual que el paquete ROS 2, evitando conflictos de nombres.

2. Verifica que `setup.py` registre **los dos nodos** como ejecutables (`entry_points`). Debe verse así:

   ```python
   entry_points={
       'console_scripts': [
           'reactive_gap_follow_node = reactive_gap_follow.reactive_gap_follow_node:main',
           'dynamic_obstacles_node = reactive_gap_follow.dynamic_obstacles_node:main',
       ],
   },
   ```

   Si falta la línea de `dynamic_obstacles_node`, agrégala con `nano setup.py` antes de compilar.

3. Regresa a la raíz de tu workspace y compila el paquete:

   ```bash
   cd ~/F1Tenth-Repository
   colcon build --packages-select reactive_gap_follow
   source install/setup.bash
   ```

   > Se compila dentro del mismo workspace del simulador (`F1Tenth-Repository`), no en un workspace separado — así se evita tener que sourcear dos workspaces distintos en una compu nueva.

---

# 🟦 PARTE 1 — 10 Vueltas sin Obstáculos

## 🗺️ 3. Configurar el mapa limpio de BrandsHatch

### 3.1 — Copiar el mapa a la carpeta del simulador

```bash
cp ~/F1Tenth-Repository/src/reactive_gap_follow/maps/BrandsHatch_map.png ~/F1Tenth-Repository/src/f1tenth_gym_ros/maps/
cp ~/F1Tenth-Repository/src/reactive_gap_follow/maps/BrandsHatch_map.yaml ~/F1Tenth-Repository/src/f1tenth_gym_ros/maps/
```

### 3.2 — Editar `sim.yaml`

```bash
nano ~/F1Tenth-Repository/src/f1tenth_gym_ros/config/sim.yaml
```

Deja la sección de mapa y agentes así (un solo auto, mapa limpio):

```yaml
# map parameters
map_path: '/home/TU_USUARIO/F1Tenth-Repository/src/f1tenth_gym_ros/maps/BrandsHatch_map'
map_img_ext: '.png'

# opponent parameters
num_agent: 1

# ego starting pose on map
sx: 0.0
sy: 0.0
stheta: 0.0
```

> Reemplaza `TU_USUARIO` por el usuario real de Linux en la máquina donde se revise el proyecto.

### 3.3 — Recompilar el simulador tras el cambio

```bash
cd ~/F1Tenth-Repository
colcon build --packages-select f1tenth_gym_ros
source install/setup.bash
```

> Este paso se repite cada vez que se edite `sim.yaml` (cambio de mapa, `num_agent`, poses de arranque, etc.).

---

## 🚀 4. Ejecución — Parte 1

Dos terminales:

### Terminal 1 — Simulador

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Se abrirá RViz mostrando la pista de BrandsHatch limpia, con un solo auto.

### Terminal 2 — Controlador (Follow the Gap)

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run reactive_gap_follow reactive_gap_follow_node
```

El vehículo comenzará a moverse automáticamente. En consola se ve el contador de vueltas y el cronómetro en tiempo real:

```
📍 Línea de meta fijada en (0.00, 0.00).
⏱️  [Vuelta 1/10] Tiempo: 48.42s | Mejor: 48.42s
⏱️  [Vuelta 2/10] Tiempo: 48.98s | Mejor: 48.98s
...
🏁 COMPETENCIA TERMINADA. Mejor vuelta: 48.30s
```

---

## 🧠 5. Enfoque técnico — Parte 1

El nodo `reactive_gap_follow_node.py` implementa telemetría integrada y procesamiento de LiDAR optimizado para BrandsHatch:

- **Campo de visión (FOV) restringido:** limita el rango útil del LiDAR al frente del vehículo, evitando que datos laterales/traseros distorsionen la elección del gap, y previniendo giros erróneos en las horquillas cerradas.
- **Burbuja de seguridad:** proyecta una zona de exclusión alrededor del obstáculo (pared) más cercano, obligando al vehículo a mantenerse alejado de los bordes de la pista a altas velocidades.
- **Selección del "deepest point":** en lugar de apuntar al centro geométrico del hueco libre, el algoritmo busca el índice más profundo (`np.argmax`) dentro del gap más ancho encontrado.
- **Suavizado exponencial (EMA):** mezcla el ángulo de dirección nuevo con el anterior, evitando correcciones bruscas del volante cerca de los bordes de la pista.
- **Perfil de velocidad dinámico** según el ángulo de giro objetivo (recta, curva suave, curva media, horquilla).
- **Telemetría de competencia:** contador automático de vueltas (hasta 10) y cronómetro por vuelta, con una guarda de tiempo mínimo para evitar falsas detecciones de vuelta cerca del punto de partida.

## 🎥 6. Video de evidencia — Parte 1

👉 [Ver video en YouTube](https://youtu.be/hUd8qSooDsQ)

---

# 🟥 PARTE 2 — Evasión de Obstáculos + Oponente Dinámico

Sobre la misma pista de BrandsHatch, ahora con **5 obstáculos estáticos** y un **segundo vehículo oponente** controlado por su propio nodo Follow the Gap.

## 🗺️ 7. Configurar el mapa con obstáculos y el segundo agente

### 7.1 — Copiar el mapa de obstáculos a la carpeta del simulador

```bash
cp ~/F1Tenth-Repository/src/reactive_gap_follow/maps/BrandsHatch_obstacles_map.png ~/F1Tenth-Repository/src/f1tenth_gym_ros/maps/
cp ~/F1Tenth-Repository/src/reactive_gap_follow/maps/BrandsHatch_obstacles_map.yaml ~/F1Tenth-Repository/src/f1tenth_gym_ros/maps/
```

### 7.2 — Editar `sim.yaml`

```bash
nano ~/F1Tenth-Repository/src/f1tenth_gym_ros/config/sim.yaml
```

Deja la sección de mapa y agentes así — esta es la configuración **ya probada y funcional**, con el oponente arrancando en una posición válida de la pista, sin chocar contra el ego ni contra los muros:

```yaml
# map parameters
map_path: '/home/TU_USUARIO/F1Tenth-Repository/src/f1tenth_gym_ros/maps/BrandsHatch_obstacles_map'
map_img_ext: '.png'

# opponent parameters
num_agent: 2

# ego starting pose on map
sx: 0.0
sy: 0.0
stheta: 0.0

# opp starting pose on map
sx1: 19.0
sy1: -20.0
stheta1: 0.0
```

> Reemplaza `TU_USUARIO` por el usuario real de Linux en la máquina donde se revise el proyecto. Los valores de `sx1`/`sy1` **no cambian** entre computadoras — son coordenadas del mapa, no del sistema de archivos.

### 7.3 — Recompilar el simulador tras el cambio

```bash
cd ~/F1Tenth-Repository
colcon build --packages-select f1tenth_gym_ros
source install/setup.bash
```

---

## 🚀 8. Ejecución — Parte 2

Tres terminales.

### Terminal 1 — Simulador

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Se abrirá RViz mostrando BrandsHatch con los 5 obstáculos y **dos** autos (ego y oponente).

### Terminal 2 — Auto principal (ego)

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run reactive_gap_follow reactive_gap_follow_node
```

### Terminal 3 — Auto oponente (obstáculo dinámico)

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run reactive_gap_follow dynamic_obstacles_node
```

En Terminal 2 se ve la misma telemetría de vueltas de la Parte 1, ahora corriendo sobre la pista con obstáculos y con el segundo auto en movimiento.

---

## 🧠 9. Enfoque técnico — Parte 2

### 9.1 — Auto principal (`reactive_gap_follow_node.py`)

Usa la misma base de Follow the Gap de la Parte 1, con ajustes para obstáculos:

- **Disparity Extender:** al detectar un salto brusco de distancia (`DISPARITY_TH = 0.30 m`) entre lecturas consecutivas del LiDAR, "extiende" el punto más cercano hacia el lado del obstáculo, cubriendo un ángulo proporcional al ancho del auto (`CAR_WIDTH/2 + margen`). Evita que el algoritmo intente pasar por un hueco más angosto que el auto — es la corrección clave que resolvió los choques por corte de curva contra el obstáculo o el muro interior.
- **FOV de 110°** y **burbuja de seguridad de 0.40 m** alrededor del punto más cercano.
- **Margen extra de 0.15 m** en el cálculo del disparity extender, para compensar la inercia del auto a alta velocidad al esquivar obstáculos.
- **Suavizado EMA simple** (`EMA_ALPHA = 0.35`), sin limitador de tasa de giro adicional — combinar ambos mecanismos generaba trompos, así que se mantiene solo uno.
- **Perfil de velocidad dinámico:** recta → 7.2 m/s, curva suave → 4.6 m/s, curva media → 2.8 m/s, horquilla → 1.5 m/s.

### 9.2 — Auto oponente (`dynamic_obstacles_node.py`)

Reutiliza exactamente la misma lógica (disparity extender + gap más profundo + EMA) pero:

- Escucha `/opp_scan` y publica en `/opp_drive` — los tópicos por defecto del segundo agente del simulador (ver `sim.yaml`, sección `opp_scan_topic` / `opp_drive_topic`), por lo que no requiere ninguna modificación al puente del simulador.
- FOV más angosto (100°) y rango de LiDAR más corto (5.0 m), suficiente para ir a velocidad moderada.
- Techo de velocidad bajo (máx. 6.0 m/s en recta, 1.8 m/s en horquilla) para actuar como obstáculo dinámico predecible y no chocar por su cuenta.

---

## 🎥 10. Video de evidencia — Parte 2

👉 [Ver video en YouTube](https://youtu.be/xCngN90NOUA)


---

## 🔁 11. Alternar entre Parte 1 y Parte 2

Ambos `.yaml` de mapa comparten la misma `resolution` y `origin`, así que alternar es solo cuestión de editar `sim.yaml`:

| | Parte 1 (limpia) | Parte 2 (obstáculos + oponente) |
|---|---|---|
| `map_path` | `.../maps/BrandsHatch_map` | `.../maps/BrandsHatch_obstacles_map` |
| `num_agent` | `1` | `2` |
| `sx1`/`sy1`/`stheta1` | *(no aplica)* | `19.0` / `-20.0` / `0.0` |

Tras cualquier cambio, recompilar `f1tenth_gym_ros` (secciones 3.3 / 7.3) y volver a sourcear.

