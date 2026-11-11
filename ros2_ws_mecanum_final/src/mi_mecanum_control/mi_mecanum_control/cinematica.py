#!/usr/bin/env python3
"""
cinematica.py — Cinemática del robot Mecanum (notación de la clase)

    ξ = [u, v, r]ᵀ                   velocidades en el marco del robot
    ω = [ω_di, ω_dd, ω_ti, ω_td]ᵀ    ruedas: delantera izq/der, trasera izq/der
    η = [x, y, φ]ᵀ                   pose en el marco odom

    Inversa : ω = Q ξ          Q de 4x3   (a = radio, lx = wheelbase/2, ly = track/2)
    Directa : ξ = W ω          W = (QᵀQ)⁻¹Qᵀ  (Q no es cuadrada -> pseudoinversa izquierda)
    Odometría: η̇ = J(φ) ξ      integrada con Euler

    Rodillos en "X" (vista superior). Con k = lx + ly, cada rueda cumple:
        ω_di = (u - v - k·r) / a        ω_dd = (u + v + k·r) / a
        ω_ti = (u + v - k·r) / a        ω_td = (u - v + k·r) / a

Lazo:
    /cmd_vel --Q--> /vel_rueda_* --> Gazebo --> ruedas
    /joint_states --W--> ξ --J(φ), Euler--> /odom + TF odom -> base_link
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster

RUEDAS = ['del_izq', 'del_der', 'tra_izq', 'tra_der']
JOINTS = [f'joint_rueda_{p}' for p in RUEDAS]
TOPICOS = [f'/vel_rueda_{p}' for p in RUEDAS]


def matriz_Q(a, lx, ly):
    """Cinemática inversa: ω = Q ξ  (4x3)."""
    # TODO (C-1): matriz Q de 4x3 a partir de las ecuaciones de cada rueda
    return np.zeros((4, 3))


def matriz_W(Q):
    """Cinemática directa: ξ = W ω  (3x4), pseudoinversa izquierda de Q."""
    # TODO (C-2): W = (QᵀQ)⁻¹ Qᵀ
    return np.zeros((3, 4))


def matriz_J(phi):
    """η̇ = J(φ) ξ : lleva las velocidades del marco del robot al marco odom (3x3)."""
    # TODO (C-3): matriz J(φ) de 3x3
    return np.zeros((3, 3))


def envolver(a):
    """Envuelve un ángulo a [-π, π]."""
    return math.atan2(math.sin(a), math.cos(a))


def integrar(eta, xi, dt):
    """Euler: η(k+1) = η(k) + J(φ) ξ dt."""
    eta = np.asarray(eta, dtype=float) + matriz_J(eta[2]) @ np.asarray(xi, dtype=float) * dt
    eta[2] = envolver(eta[2])
    return eta


class Cinematica(Node):
    def __init__(self):
        super().__init__('cinematica')
        a = self.declare_parameter('rueda_radio', 0.05).value
        lx = self.declare_parameter('distancia_ejes', 0.30).value / 2.0
        ly = self.declare_parameter('separacion_ruedas', 0.28).value / 2.0

        self.Q = matriz_Q(a, lx, ly)
        self.W = matriz_W(self.Q)

        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel, 10)
        self.pubs = [self.create_publisher(Float64, t, 10) for t in TOPICOS]
        self.create_subscription(JointState, '/joint_states', self.cb_joints, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)

        self.eta = np.zeros(3)
        self.t_ant = None

        np.set_printoptions(precision=4, suppress=True)
        self.get_logger().info(
            f'a={a} m, lx={lx} m, ly={ly} m\nQ =\n{self.Q}\nW =\n{self.W}\nW·Q =\n{self.W @ self.Q}')

    # ---------- Cinemática inversa: ω = Q ξ ----------
    def cb_cmd_vel(self, msg):
        xi = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        for pub, w in zip(self.pubs, self.Q @ xi):
            pub.publish(Float64(data=float(w)))

    # ---------- Cinemática directa: ξ = W ω, y odometría ----------
    def cb_joints(self, msg):
        try:
            omega = np.array([msg.velocity[msg.name.index(j)] for j in JOINTS])
        except (ValueError, IndexError):
            return
        ahora = self.get_clock().now()
        if self.t_ant is None:
            self.t_ant = ahora
            return
        dt = (ahora - self.t_ant).nanoseconds * 1e-9
        self.t_ant = ahora
        if dt <= 0.0:
            return

        xi = self.W @ omega
        self.eta = integrar(self.eta, xi, dt)
        x, y, phi = (float(c) for c in self.eta)
        qz, qw = math.sin(phi / 2.0), math.cos(phi / 2.0)
        stamp = ahora.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = float(xi[0])
        odom.twist.twist.linear.y = float(xi[1])
        odom.twist.twist.angular.z = float(xi[2])
        self.pub_odom.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    nodo = Cinematica()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
