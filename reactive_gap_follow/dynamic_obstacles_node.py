#!/usr/bin/env python3
"""
Nodo ROS 2: Dynamic Obstacles Controller (Inteligente y Seguro)
===============================================================
Controla al vehículo oponente (opp_racecar_0) a velocidad moderada.
Utiliza exactamente la misma lógica probada de Follow the Gap del 
vehículo principal (incluyendo el extensor de disparidad y correcciones 
trigonométricas) pero con un límite de velocidad conservador para no 
chocar jamás y servir como un obstáculo dinámico predecible.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class DynamicObstacles(Node):
    def __init__(self):
        super().__init__('dynamic_obstacles_node')

        # ── LiDAR y Vehículo (Enemigo - Lento y Seguro) ──────────
        self.MAX_RANGE = 5.0     # Ve corto, es suficiente para ir a 2m/s
        self.FOV_DEG   = 100.0   # FOV equilibrado
        self.CAR_WIDTH = 0.50    # Su propio ancho
        self.BUBBLE_R  = 0.35    # Burbuja segura
        self.DISPARITY_TH = 0.30

        # ── Steering y Suavizado ─────────────────────────────────
        self.MAX_STEER = 0.40    # rad
        self.EMA_ALPHA = 0.35    
        self._prev_steer = 0.0

        # Suscripción al tópico del enemigo
        self.create_subscription(LaserScan, '/opp_scan', self._cb_scan_0, 10)
        self.pub_0 = self.create_publisher(AckermannDriveStamped, '/opp_drive', 10)

        self.get_logger().info('🤖 Oponente Dinámico ACTIVADO. Mismo cerebro, menos velocidad.')

    # ════════════════════════════════════════════════════════════════
    # LÓGICA PRINCIPAL (Idéntica a tu carro principal)
    # ════════════════════════════════════════════════════════════════
    def _cb_scan_0(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.where(np.isinf(ranges) | np.isnan(ranges), self.MAX_RANGE, ranges)
        ranges = np.clip(ranges, 0.0, self.MAX_RANGE)
        
        proc, start_idx = self._apply_fov(ranges, msg)

        # Inflar los bordes de los obstáculos
        proc = self._disparity_ext(proc, msg.angle_increment)

        # Burbuja adicional
        closest_idx = int(np.argmin(proc))
        proc = self._bubble(proc, closest_idx, proc[closest_idx], msg.angle_increment)

        # Encontrar el gap más grande y apuntar al punto más profundo
        best_angle = self._find_best_gap(proc, start_idx, msg)

        # Suavizar dirección
        raw_angle = float(np.clip(best_angle, -self.MAX_STEER, self.MAX_STEER))
        angle = self.EMA_ALPHA * raw_angle + (1 - self.EMA_ALPHA) * self._prev_steer
        self._prev_steer = angle

        # Perfil de Velocidad (Lento)
        speed = self._calculate_speed(angle)

        # Publicar
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.drive.steering_angle = float(angle)
        cmd.drive.speed = float(speed)
        self.pub_0.publish(cmd)

    # ════════════════════════════════════════════════════════════════
    # DISPARITY EXTENDER
    # ════════════════════════════════════════════════════════════════
    def _disparity_ext(self, r: np.ndarray, inc: float) -> np.ndarray:
        out = r.copy()
        diffs = np.diff(r)
        
        jumps = np.where(np.abs(diffs) > self.DISPARITY_TH)[0]
        
        for i in jumps:
            d1 = r[i]
            d2 = r[i+1]
            
            if d1 < d2:
                close = d1
                step = 1
                start = i + 1
            else:
                close = d2
                step = -1
                start = i
                
            safe_dist = max(close, 0.15)
            ratio = min(1.0, (self.CAR_WIDTH / 2.0 + 0.1) / safe_dist)
            angle_to_cover = math.asin(ratio)
            n_ext = int(math.ceil(angle_to_cover / inc))
            
            curr = start
            for _ in range(n_ext):
                if 0 <= curr < len(out):
                    out[curr] = min(out[curr], close)
                    curr += step
                else:
                    break
        return out

    # ════════════════════════════════════════════════════════════════
    # PROCESAMIENTO DE GAPS
    # ════════════════════════════════════════════════════════════════
    def _find_best_gap(self, proc: np.ndarray, fov_start: int, msg: LaserScan) -> float:
        non_zeros = proc > 0.0
        edges = np.diff(non_zeros.astype(int))
        starts = np.where(edges == 1)[0] + 1
        ends = np.where(edges == -1)[0]
        
        if non_zeros[0]: starts = np.insert(starts, 0, 0)
        if non_zeros[-1]: ends = np.append(ends, len(proc) - 1)

        if len(starts) == 0:
            return 0.0 

        gap_lengths = ends - starts
        max_gap_idx = np.argmax(gap_lengths)
        
        best_start = starts[max_gap_idx]
        best_end = ends[max_gap_idx]

        gap_segment = proc[best_start:best_end + 1]
        deepest_local_idx = np.argmax(gap_segment)
        deepest_global_idx = best_start + deepest_local_idx

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
        safe_dist = max(dist, 0.15)
        ratio = min(1.0, self.BUBBLE_R / safe_dist)
        angle_rad = math.asin(ratio)
        radius_pts = int(angle_rad / inc)
        
        max_pts = len(r) // 4
        radius_pts = min(radius_pts, max_pts)
        
        lo = max(0, center_idx - radius_pts)
        hi = min(len(r), center_idx + radius_pts + 1)
        r[lo:hi] = 0.0
        return r

    # ════════════════════════════════════════════════════════════════
    # PERFIL DE VELOCIDAD (LENTO PARA OBSTÁCULO)
    # ════════════════════════════════════════════════════════════════
    def _calculate_speed(self, angle: float) -> float:
        deg = math.degrees(abs(angle))
        
        if deg < 10.0:
            return 3.0    # Recta: Paseando
        elif deg < 20.0:
            return 1.5    # Curva suave
        else:
            return 1.0    # Horquilla: Muy lento

def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
