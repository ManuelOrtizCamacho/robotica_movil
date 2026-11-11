#!/usr/bin/env python3
"""
navegar_astar.py — Planea con A* y lleva el robot hasta la meta del carné (DADO).

    ros2 run mi_mecanum_control navegar_astar --ros-args -p carne:=<carne> -p use_sim_time:=true

1. Construye la grilla del mundo (mapa.py) y corre A* (astar.py) desde (0, 0) hasta la meta.
2. Publica el mapa en /mapa y el camino en /plan (para verlos en RViz).
3. Sigue los waypoints con la ley de control a puntos (control.py) publicando /cmd_vel.
4. Guarda resultado_<carne>.csv con la odometría y la distancia mínima del LiDAR.
"""
import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan

from mi_mecanum_control.datos_carne import datos, texto
from mi_mecanum_control.mapa import (RES, N, LADO, construir_grilla, a_celda,
                                     a_metros, simplificar, libre)
from mi_mecanum_control.astar import astar
from mi_mecanum_control.control import ley_control


class NavegarAStar(Node):
    def __init__(self):
        super().__init__('navegar_astar')
        carne = self.declare_parameter('carne', '').value
        self.D = datos(carne)
        self.get_logger().info('\n' + texto(self.D))

        # ---------- A* ----------
        self.grilla = construir_grilla()
        inicio, meta = a_celda(0.0, 0.0), a_celda(*self.D['meta'])
        camino, costo = astar(self.grilla, inicio, meta)
        self.costo_m = costo * RES
        if not camino or camino[0] != inicio or camino[-1] != meta:
            self.get_logger().error('A* no devolvió un camino válido de inicio a meta. Revise astar.py')
            camino = []
        self.camino = camino
        wps = simplificar(self.grilla, camino) if camino else []
        th = self.D['theta_meta']
        self.puntos = [(*a_metros(c), th) for c in wps[1:]]
        if self.puntos:
            self.puntos[-1] = (*self.D['meta'], th)
        self.get_logger().info(
            f'A*: {len(camino)} celdas | costo = {self.costo_m:.3f} m | '
            f'waypoints = {[(round(p[0], 2), round(p[1], 2)) for p in self.puntos]}')

        # ---------- Publicadores para RViz ----------
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_mapa = self.create_publisher(OccupancyGrid, '/mapa', qos)
        self.pub_plan = self.create_publisher(Path, '/plan', qos)
        self.publicar_mapa_y_plan()

        # ---------- Control ----------
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.cb_scan, 10)
        self.pose = None
        self.d_min_scan = float('nan')
        self.i = 0
        self.t0 = None
        self.terminado = False

        # ---------- Registro ----------
        self.ruta_csv = os.path.abspath(f"resultado_{self.D['carne']}.csv")
        self.f = open(self.ruta_csv, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow([f"# carne={self.D['carne']}", datetime.now().isoformat(timespec='seconds'),
                         f'meta={self.D["meta"]}', f'costo_astar_m={self.costo_m:.3f}'])
        self.w.writerow(['t', 'x', 'y', 'phi', 'd_min_lidar', 'waypoint'])
        self.create_timer(0.02, self.lazo)

    def publicar_mapa_y_plan(self):
        stamp = self.get_clock().now().to_msg()
        m = OccupancyGrid()
        m.header.frame_id = 'odom'
        m.header.stamp = stamp
        m.info.resolution = RES
        m.info.width = N
        m.info.height = N
        m.info.origin.position.x = -LADO / 2
        m.info.origin.position.y = -LADO / 2
        m.data = [100 if self.grilla[j][i] else 0 for j in range(N) for i in range(N)]
        self.pub_mapa.publish(m)

        p = Path()
        p.header.frame_id = 'odom'
        p.header.stamp = stamp
        for c in self.camino:
            ps = PoseStamped()
            ps.header = p.header
            ps.pose.position.x, ps.pose.position.y = a_metros(c)
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        self.pub_plan.publish(p)

    def cb_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        phi = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        self.pose = (p.x, p.y, phi)

    def cb_scan(self, msg):
        validos = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        self.d_min_scan = min(validos) if validos else float('nan')

    def lazo(self):
        if self.pose is None or self.terminado:
            return
        ahora = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = ahora
        t = ahora - self.t0
        cmd = Twist()
        if self.i < len(self.puntos):
            final = self.i == len(self.puntos) - 1
            u, v, r, ok = ley_control(self.pose, self.puntos[self.i], final)
            if ok:
                self.get_logger().info(f'Waypoint {self.i + 1}/{len(self.puntos)} alcanzado en t = {t:.1f} s')
                self.i += 1
            cmd.linear.x, cmd.linear.y, cmd.angular.z = u, v, r
        else:
            x, y, phi = self.pose
            ex = math.hypot(self.D['meta'][0] - x, self.D['meta'][1] - y)
            self.get_logger().info(
                f'META ALCANZADA | carné {self.D["carne"]} | tiempo = {t:.1f} s | '
                f'costo A* = {self.costo_m:.3f} m | error final = {ex * 1000:.1f} mm | CSV: {self.ruta_csv}')
            self.terminado = True
            self.f.flush()
        self.pub_cmd.publish(cmd)
        x, y, phi = self.pose
        self.w.writerow([f'{t:.3f}', f'{x:.4f}', f'{y:.4f}', f'{phi:.4f}',
                         f'{self.d_min_scan:.3f}', self.i])


def main():
    rclpy.init()
    nodo = NavegarAStar()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.pub_cmd.publish(Twist())
        nodo.f.close()
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
