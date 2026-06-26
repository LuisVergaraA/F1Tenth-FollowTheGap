#!/usr/bin/env python3
"""
Nodo ROS 2: ReactiveFollowGap — BrandsHatch Optimizado
======================================================
Objetivo: 10 vueltas sin colisión + Menor tiempo posible.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('reactive_follow_gap_node')

        # ── LiDAR y Vehículo ─────────────────────────────────────
        self.MAX_RANGE = 6.0     # Rango útil del LiDAR
        self.FOV_DEG   = 90.0   # Campo de visión frontal (70° izq, 70° der)
        self.CAR_WIDTH = 0.40    # Ancho del carro con margen
        self.BUBBLE_R  = 0.75    # Radio de la burbuja de seguridad

        # ── Steering y Suavizado ─────────────────────────────────
        self.MAX_STEER = 0.35    # rad
        self.EMA_ALPHA = 0.30     # Filtro paso bajo para suavizar la dirección
        self._prev_steer = 0.0

        # ── Telemetría y Competencia ─────────────────────────────
        self.TOTAL_LAPS = 10
        self.FINISH_R   = 1.5    # Radio de detección de la meta
        self.ARM_DIST   = 3.0    # Distancia para armar el trigger de la meta
        self.MIN_LAP_T  = 5.0    # Tiempo mínimo lógico por vuelta
        
        self._start_pos  = None
        self._left_start = False
        self._lap_count  = 0
        self._lap_t0     = None
        self._best_lap   = float('inf')
        self._race_done  = False

        # ── ROS 2 Pubs/Subs ──────────────────────────────────────
        self.create_subscription(LaserScan, '/scan', self._cb_scan, 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self._cb_odom, 10)
        self._pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.get_logger().info('🏁 Follow the Gap: Listo para BrandsHatch.')

    # ════════════════════════════════════════════════════════════════
    # LÓGICA PRINCIPAL DEL LIDAR
    # ════════════════════════════════════════════════════════════════
    def _cb_scan(self, msg: LaserScan):
        if self._race_done:
            self._pub_cmd(0.0, 0.0) # Detener el vehículo
            return

        # 1. Limpiar datos y recortar FOV
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.where(np.isinf(ranges) | np.isnan(ranges), self.MAX_RANGE, ranges)
        ranges = np.clip(ranges, 0.0, self.MAX_RANGE)
        
        proc, start_idx = self._apply_fov(ranges, msg)

        # 2. Encontrar el obstáculo más cercano y crear la burbuja
        closest_idx = int(np.argmin(proc))
        closest_dist = proc[closest_idx]
        proc = self._bubble(proc, closest_idx, closest_dist, msg.angle_increment)

        # 3. Encontrar el gap más grande y apuntar al punto más profundo
        best_angle = self._find_best_gap(proc, start_idx, msg)

        # 4. Suavizar dirección
        raw_angle = float(np.clip(best_angle, -self.MAX_STEER, self.MAX_STEER))
        angle = self.EMA_ALPHA * raw_angle + (1 - self.EMA_ALPHA) * self._prev_steer
        self._prev_steer = angle

        # 5. Perfil de Velocidad Dinámico
        speed = self._calculate_speed(angle)

        self._pub_cmd(angle, speed)

    # ════════════════════════════════════════════════════════════════
    # PROCESAMIENTO DE GAPS (Optimizado)
    # ════════════════════════════════════════════════════════════════
    def _find_best_gap(self, proc: np.ndarray, fov_start: int, msg: LaserScan) -> float:
        # Encontrar secuencias de puntos > 0
        non_zeros = proc > 0.0
        # Truco de NumPy para encontrar islas contiguas de valores True
        edges = np.diff(non_zeros.astype(int))
        starts = np.where(edges == 1)[0] + 1
        ends = np.where(edges == -1)[0]
        
        if non_zeros[0]: starts = np.insert(starts, 0, 0)
        if non_zeros[-1]: ends = np.append(ends, len(proc) - 1)

        if len(starts) == 0:
            return 0.0 # Si no hay gaps, ir recto ciegamente (emergencia)

        # Encontrar el gap más ancho (mayor número de índices)
        gap_lengths = ends - starts
        max_gap_idx = np.argmax(gap_lengths)
        
        best_start = starts[max_gap_idx]
        best_end = ends[max_gap_idx]

        # En lugar de ir al centro, apuntamos al punto más profundo (lejano) del mejor gap
        gap_segment = proc[best_start:best_end + 1]
        deepest_local_idx = np.argmax(gap_segment)
        deepest_global_idx = best_start + deepest_local_idx

        # Calcular el ángulo real
        real_idx = fov_start + deepest_global_idx
        return msg.angle_min + real_idx * msg.angle_increment

    # ════════════════════════════════════════════════════════════════
    # HELPERS
    # ════════════════════════════════════════════════════════════════
    def _apply_fov(self, r: np.ndarray, msg: LaserScan):
        fov_rad = np.deg2rad(self.FOV_DEG)
        total_rad = msg.angle_max - msg.angle_min
        if fov_rad >= total_rad:
            return r.copy(), 0
        
        num_pts = int(fov_rad / msg.angle_increment)
        center = len(r) // 2
        start = max(0, center - num_pts // 2)
        end = min(len(r), center + num_pts // 2)
        return r[start:end].copy(), start

    def _bubble(self, r: np.ndarray, center_idx: int, dist: float, inc: float) -> np.ndarray:
        safe_dist = max(dist, 0.1)
        # Cuántos índices abarcan el radio de la burbuja a esta distancia
        radius_pts = int(math.ceil(self.BUBBLE_R / (safe_dist * inc)))
        
        lo = max(0, center_idx - radius_pts)
        hi = min(len(r), center_idx + radius_pts + 1)
        r[lo:hi] = 0.0
        return r

    def _calculate_speed(self, angle: float) -> float:
        # Velocidad atada directamente a la severidad de la curva
        deg = math.degrees(abs(angle))
        if deg < 6.0:
            return 7.5  # Recta: Velocidad máxima
        elif deg < 10.0:
            return 3.5  # Curva leve
        elif deg < 15.0:
            return 2.5  # Curva pronunciada
        else:
            return 1.3  # Horquilla / U-turn (Frenado fuerte)

    def _pub_cmd(self, steer: float, speed: float):
        m = AckermannDriveStamped()
        # Tiempo correcto de ROS 2
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.drive.steering_angle = float(steer)
        m.drive.speed = float(speed)
        self._pub.publish(m)

    # ════════════════════════════════════════════════════════════════
    # TELEMETRÍA (10 Vueltas y Mejor Tiempo)
    # ════════════════════════════════════════════════════════════════
    def _cb_odom(self, msg: Odometry):
        if self._race_done:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Obtener el tiempo de simulación en segundos
        current_time = self.get_clock().now().nanoseconds / 1e9 

        if self._start_pos is None:
            self._start_pos = (x, y)
            self._lap_t0 = current_time
            self.get_logger().info(f'📍 Línea de meta fijada en ({x:.2f}, {y:.2f}).')
            return

        dist_to_start = math.hypot(x - self._start_pos[0], y - self._start_pos[1])

        # Armar el trigger cuando nos alejamos de la meta
        if not self._left_start and dist_to_start > self.ARM_DIST:
            self._left_start = True

        # Disparar cuando volvemos a entrar al radio de la meta
        if self._left_start and dist_to_start <= self.FINISH_R:
            elapsed = current_time - self._lap_t0
            if elapsed >= self.MIN_LAP_T:
                self._lap_count += 1
                if elapsed < self._best_lap:
                    self._best_lap = elapsed
                
                self.get_logger().info(
                    f'⏱️ [Vuelta {self._lap_count}/{self.TOTAL_LAPS}] '
                    f'Tiempo: {elapsed:.2f}s | Mejor: {self._best_lap:.2f}s'
                )
                
                self._lap_t0 = current_time
                self._left_start = False

                if self._lap_count >= self.TOTAL_LAPS:
                    self._race_done = True
                    self.get_logger().info(f'🏁 COMPETENCIA TERMINADA. Mejor vuelta general: {self._best_lap:.2f}s')

def main(args=None):
    rclpy.init(args=args)
    node = ReactiveFollowGap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
