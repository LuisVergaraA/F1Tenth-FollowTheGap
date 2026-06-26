# 🏎️ F1Tenth - Controlador Reactivo (Follow the Gap)

Este repositorio contiene la implementación de un controlador reactivo basado en el algoritmo **Follow the Gap** para el simulador F1Tenth en ROS 2. El algoritmo fue optimizado específicamente para completar 10 vueltas consecutivas sin colisiones en la pista de **BrandsHatch**, logrando un mejor tiempo de 47.86s.

## 🧠 Descripción del Enfoque

El controlador utiliza los datos del escáner LiDAR (`/scan`) para evadir obstáculos y encontrar la ruta óptima de navegación en tiempo real. Las modificaciones clave incluyen:

* **Restricción del Campo de Visión (FOV a 90°):** Para evitar que el vehículo realice giros en "U" en curvas cerradas, el campo de visión se limitó a 45° a cada lado. Esto actúa como "anteojeras", forzando al vehículo a buscar siempre la salida hacia adelante.
* **Burbuja de Seguridad Dinámica (0.75m):** Se crea una zona de exclusión alrededor del punto más cercano detectado por el LiDAR, proyectando el obstáculo para evitar roces laterales en las curvas de alta velocidad.
* **Selección del "Punto más Profundo":** En lugar de dirigirse al centro geométrico del espacio libre (gap), el algoritmo busca el índice más profundo (`np.argmax`) dentro del gap más ancho, garantizando una línea de carrera más natural y rápida.
* **Perfil de Velocidad basado en Dirección:** La velocidad se asigna de manera inversamente proporcional al ángulo de giro. En rectas (ángulo < 6°) acelera a 7.5 m/s, mientras que en horquillas (> 15°) frena a 1.3 m/s para mantener la tracción.

## 📂 Estructura del Código

El nodo principal está contenido en `reactive_follow_gap_node.py` e incluye los siguientes métodos principales:

1.  `_cb_scan`: Bucle principal que preprocesa el LiDAR, aplica la burbuja, encuentra el mejor gap y publica el comando de conducción.
2.  `_find_best_gap`: Lógica de NumPy para aislar arreglos contiguos de espacio libre y encontrar el índice objetivo.
3.  `_calculate_speed`: Perfil dinámico de aceleración y frenado condicionado al `steering_angle`.
4.  `_cb_odom`: Sistema integrado de telemetría. Detecta el inicio de la carrera mediante coordenadas, arma un "trigger" al alejarse 3.0m y registra una nueva vuelta al regresar al radio de meta (1.5m) después de un tiempo mínimo, calculando y almacenando el "Mejor Tiempo".

## 🚀 Instrucciones de Ejecución

1. Clonar el repositorio dentro de tu workspace de ROS 2:
   ```bash
   cd ~/f1tenth_ws/src
   git clone https://github.com/LuisVergaraA/F1Tenth-FollowTheGap
