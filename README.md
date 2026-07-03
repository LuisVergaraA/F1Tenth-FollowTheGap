# 🏎️ F1Tenth - Controlador Reactivo (Follow the Gap)

Este repositorio contiene la implementación individual de un paquete de ROS 2 (Humble) con un controlador reactivo basado en el algoritmo **Follow the Gap** para el simulador oficial de F1Tenth.

El algoritmo fue diseñado y optimizado para superar el reto de completar **10 vueltas consecutivas sin colisiones** en la pista técnica de **BrandsHatch**, logrando tiempos competitivos (mejor vuelta: ~47.86s).

---

## 🛠️ 1. Instalación y Compilación

Se asume que el evaluador ya cuenta con el workspace oficial del simulador F1Tenth compilado en su máquina local.

1. Dirígete a la carpeta `src` de tu workspace de F1Tenth (por ejemplo, `~/f1tenth_ws/src` o `~/F1Tenth-Repository/src`):

   ```bash
   cd ~/TU_WORKSPACE_F1TENTH/src
   git clone https://github.com/LuisVergaraA/F1Tenth-FollowTheGap.git
   ```

2. Regresa a la raíz de tu workspace y compila únicamente este paquete:

   ```bash
   cd ~/TU_WORKSPACE_F1TENTH
   colcon build --packages-select reactive_gap_follow
   ```

---

## 🗺️ 2. Nota sobre el Mapa (BrandsHatch)

Antes de lanzar el simulador, es estrictamente necesario actualizar la ruta absoluta del mapa para que apunte a BrandsHatch y coincida con el nombre de usuario de tu máquina local.

1. Abre el archivo de configuración `sim.yaml`:

   ```bash
   # generalmente asi:
   nano ~/F1Tenth-Repository/src/f1tenth_gym_ros/config/sim.yaml
   ```

   > **Nota:** si la carpeta de configuración en tu versión se llama `params`, ajusta la ruta a `src/f1tenth_gym_ros/params/sim.yaml`.

2. Localiza el parámetro `map_path` y modifícalo para incluir tu usuario de Linux y el nombre del mapa BrandsHatch:

   ```yaml
   # Reemplaza 'tu_usuario' por el nombre de tu usuario en Ubuntu
   map_path: '/home/tu_usuario/F1Tenth-Repository/src/f1tenth_gym_ros/maps/BrandsHatch'
   ```

---

## 🚀 3. Instrucciones de Ejecución

Para evaluar el proyecto, se deben ejecutar el simulador y el controlador en dos terminales separadas.

### Terminal 1: Lanzar el Simulador

Configura el entorno en tu workspace e inicia el simulador (se abrirá la ventana de RViz con la pista de BrandsHatch):

```bash
cd ~/TU_WORKSPACE_F1TENTH
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### Terminal 2: Ejecutar el Controlador (Follow the Gap)

Abre una nueva pestaña o terminal, configura el entorno nuevamente y ejecuta el nodo de conducción autónoma:

```bash
cd ~/TU_WORKSPACE_F1TENTH
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run reactive_gap_follow reactive_gap_follow_node
```

---

## 🧠 4. Enfoque Técnico y Optimizaciones

El nodo `reactive_gap_follow_node.py` implementa telemetría integrada y un procesamiento LiDAR avanzado para dominar las curvas críticas de BrandsHatch:

- **Restricción de visión (FOV 90°):** para evitar que el vehículo gire en sentido contrario en las horquillas cerradas (U-turns), el FOV se limitó a 45° a cada lado.
- **Burbuja anti-derrape (0.75 m):** proyecta una amplia zona de exclusión alrededor del obstáculo más cercano, obligando al vehículo a mantenerse en el centro de la pista a altas velocidades.
- **Selección del "deepest point":** en lugar de apuntar al centro geométrico del hueco, el algoritmo busca el índice más profundo (`np.argmax`) dentro del gap más ancho.
- **Perfil de velocidad dinámico:**

  | Tipo de tramo | Ángulo de giro | Velocidad |
  |---|---|---|
  | Rectas puras | < 6° | 7.5 m/s |
  | Curvas suaves | < 10° | 3.5 m/s |
  | Curvas medias | < 15° | 2.5 m/s |
  | Horquillas | > 15° | 1.3 m/s (frenada para mantener tracción) |

---

## 🎥 5. Video de Evidencia

Demostración en video cumpliendo los requisitos (10 vueltas sin colisión, contador y cronómetro en pantalla):

👉 [Ver video en YouTube](https://youtu.be/hUd8qSooDsQ)