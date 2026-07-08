# 🏎️ F1Tenth - Controlador Reactivo (Follow the Gap)

Este repositorio contiene la implementación individual de un paquete de ROS 2 (Humble) con un controlador reactivo basado en el algoritmo **Follow the Gap** para el simulador oficial de F1Tenth.

**Parte 1 del proyecto:** el controlador está diseñado para completar **10 vueltas consecutivas sin colisiones** en la pista de **BrandsHatch** (pista limpia, sin obstáculos), logrando tiempos competitivos.

---

## ✅ Prerrequisitos

- **Ubuntu 22.04** con **ROS 2 Humble** instalado.
- El **simulador oficial de F1Tenth** ya instalado y compilado, siguiendo la guía de:
  👉 [widegonz/F1Tenth-Repository](https://github.com/widegonz/F1Tenth-Repository)

Este repositorio **no incluye el simulador en sí** — es un controlador (más el mapa de la pista) que se ejecuta *sobre* el simulador ya instalado, comunicándose por tópicos de ROS 2 (`/scan`, `/drive`, `/ego_racecar/odom`).

---

## 📦 1. Estructura de este repositorio

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
│   ├── BrandsHatch_map.png     ← mapa de la pista
│   └── BrandsHatch_map.yaml    ← metadata del mapa (resolución, origen, etc.)
└── reactive_gap_follow/
    ├── __init__.py
    └── reactive_gap_follow_node.py   ← lógica del controlador
```

Este es un paquete ROS 2 (`ament_python`) completo, más una carpeta `maps/` con el mapa oficial de BrandsHatch ya incluido. Todo lo necesario para correr la Parte 1 del proyecto está en este repositorio — no hace falta descargar el mapa de ninguna otra fuente.

---

## 🛠️ 2. Instalación y Compilación

1. Dirígete a la carpeta `src` de tu workspace del simulador F1Tenth (por ejemplo, `~/F1Tenth-Repository/src`):

   ```bash
   cd ~/F1Tenth-Repository/src
   git clone https://github.com/LuisVergaraA/F1Tenth-FollowTheGap.git reactive_gap_follow
   ```

   > ⚠️ Nota el `reactive_gap_follow` al final del `git clone` — esto asegura que la carpeta del paquete se llame igual que el paquete ROS 2 (`reactive_gap_follow`), evitando conflictos de nombres.

2. Regresa a la raíz de tu workspace y compila únicamente este paquete:

   ```bash
   cd ~/F1Tenth-Repository
   colcon build --packages-select reactive_gap_follow
   source install/setup.bash
   ```

---

## 🗺️ 3. Configurar el mapa de BrandsHatch

El mapa ya está incluido en este repositorio, dentro de `src/reactive_gap_follow/maps/`. Solo falta copiarlo a la carpeta de mapas del simulador y apuntar `sim.yaml` hacia él.

### 3.1 — Copiar el mapa a la carpeta del simulador

```bash
cp ~/F1Tenth-Repository/src/reactive_gap_follow/maps/BrandsHatch_map.png ~/F1Tenth-Repository/src/f1tenth_gym_ros/maps/
cp ~/F1Tenth-Repository/src/reactive_gap_follow/maps/BrandsHatch_map.yaml ~/F1Tenth-Repository/src/f1tenth_gym_ros/maps/
```

### 3.2 — Editar `sim.yaml` para apuntar a este mapa

```bash
nano ~/F1Tenth-Repository/src/f1tenth_gym_ros/config/sim.yaml
```

Busca el parámetro `map_path` y ajústalo con la ruta absoluta al mapa (**sin extensión**, el simulador agrega `.png`/`.yaml` automáticamente):

```yaml
map_path: '/home/TU_USUARIO/F1Tenth-Repository/src/f1tenth_gym_ros/maps/BrandsHatch_map'
```

> Reemplaza `TU_USUARIO` por tu usuario real de Linux.

Verifica también que el campo `map_img_ext` diga:

```yaml
map_img_ext: '.png'
```

Guarda y sal (`Ctrl+O` → `Enter` → `Ctrl+X`).

### 3.3 — Recompilar el simulador tras el cambio

Como se modificó un archivo de configuración de un paquete ya compilado, es necesario reconstruirlo:

```bash
cd ~/F1Tenth-Repository
colcon build --packages-select f1tenth_gym_ros
source install/setup.bash
```

> 💡 Este paso solo se hace una vez. No es necesario repetirlo para volver a correr el controlador.

---

## 🚀 4. Instrucciones de Ejecución

Se deben ejecutar el simulador y el controlador en dos terminales separadas.

### Terminal 1 — Lanzar el Simulador

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Se abrirá una ventana de RViz mostrando la pista de BrandsHatch.

### Terminal 2 — Ejecutar el Controlador (Follow the Gap)

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run reactive_gap_follow reactive_gap_follow_node
```

El vehículo comenzará a moverse automáticamente. En la consola de esta terminal verás en tiempo real el contador de vueltas y el cronómetro:

```
📍 Línea de meta fijada en (0.00, 0.00).
⏱️  [Vuelta 1/10] Tiempo: 48.42s | Mejor: 48.42s
⏱️  [Vuelta 2/10] Tiempo: 48.98s | Mejor: 48.98s
...
🏁 COMPETENCIA TERMINADA. Mejor vuelta: 48.30s
```

---

## 🧠 5. Enfoque Técnico y Optimizaciones

El nodo `reactive_gap_follow_node.py` implementa telemetría integrada y un procesamiento LiDAR optimizado para dominar las curvas críticas de BrandsHatch:

- **Campo de visión (FOV) restringido:** limita el rango útil del LiDAR al frente del vehículo, evitando que datos laterales/traseros distorsionen la elección del gap, y previniendo giros erróneos en las horquillas cerradas.
- **Burbuja de seguridad:** proyecta una zona de exclusión alrededor del obstáculo (pared) más cercano, obligando al vehículo a mantenerse alejado de los bordes de la pista a altas velocidades.
- **Selección del "deepest point":** en lugar de apuntar al centro geométrico del hueco libre, el algoritmo busca el índice más profundo (más lejano, `np.argmax`) dentro del gap más ancho encontrado.
- **Suavizado exponencial (EMA):** mezcla el ángulo de dirección nuevo con el anterior, evitando correcciones bruscas del volante cerca de los bordes de la pista.
- **Perfil de velocidad dinámico:**

  | Tipo de tramo | Ángulo de giro | Velocidad |
  |---|---|---|
  | Rectas puras | < 6° | 7.5 m/s |
  | Curvas suaves | < 10° | 3.5 m/s |
  | Curvas medias | < 15° | 2.5 m/s |
  | Horquillas | > 15° | 1.3 m/s (frenada para mantener tracción) |

- **Telemetría de competencia:** contador automático de vueltas (hasta 10) y cronómetro por vuelta, con una guarda de tiempo mínimo para evitar falsas detecciones de vuelta en secciones de la pista cercanas al punto de partida.

---

## 🎥 6. Video de Evidencia

Demostración en video cumpliendo los requisitos de la Parte 1 (10 vueltas sin colisión, contador y cronómetro en pantalla):

👉 [Ver video en YouTube](https://youtu.be/hUd8qSooDsQ)