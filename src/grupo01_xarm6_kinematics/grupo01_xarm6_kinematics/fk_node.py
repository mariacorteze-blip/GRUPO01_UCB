#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import math

class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__('fk_node')

        # Nombres de las articulaciones del xArm6 (según URDF)
        self.joint_names = [
            'joint1', 'joint2', 'joint3', 
            'joint4', 'joint5', 'joint6'
        ]

        # Suscripción al tópico /joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        self.get_logger().info('Nodo de Cinemática Directa (FK) para xArm6 iniciado.')

    def dh_matrix(self, theta, d, a, alpha):
        """Calcula la matriz de transformación D-H Estándar A_i."""
        ct = math.cos(theta)
        st = math.sin(theta)
        ca = math.cos(alpha)
        sa = math.sin(alpha)

        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [ 0,       sa,       ca,      d],
            [ 0,        0,        0,      1]
        ], dtype=np.float64)

    def rotation_matrix_to_quaternion(self, R):
        """Convierte una matriz de rotación 3x3 a un cuaternión [x, y, z, w]."""
        tr = np.trace(R)
        if tr > 0:
            S = math.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S

        return [qx, qy, qz, qw]

    def joint_states_callback(self, msg: JointState):
        """Maneja el mensaje recibido de las articulaciones."""
        q_dict = {}
        for name, pos in zip(msg.name, msg.position):
            q_dict[name] = pos

        if not all(j in q_dict for j in self.joint_names):
            return

        q1 = q_dict['joint1']
        q2 = q_dict['joint2']
        q3 = q_dict['joint3']
        q4 = q_dict['joint4']
        q5 = q_dict['joint5']
        q6 = q_dict['joint6']

        # Tabla D-H según la imagen adjunta: (theta_i, d_i, a_i, alpha_i)
        # Nota: Los ángulos en grados se convierten a radianes con math.radians()
        dh_params = [
            (q1,                       0.267,   0.0,     math.radians(-90)),
            (q2 - math.radians(79.35), 0.0,     0.28949, math.radians(0)),
            (q3 + math.radians(79.35), 0.0,     0.0775,  math.radians(-90)),
            (q4,                       0.3425,  0.0,     math.radians(90)),
            (q5,                       0.0,     0.076,   math.radians(-90)),
            (q6,                       0.097,   0.0,     math.radians(0))
        ]

        # Multiplicación matricial T_0_6 = A1 * A2 * A3 * A4 * A5 * A6
        T06 = np.identity(4, dtype=np.float64)
        for theta, d, a, alpha in dh_params:
            A_i = self.dh_matrix(theta, d, a, alpha)
            T06 = np.dot(T06, A_i)

        # Extraer posición P_3x1
        x = T06[0, 3]
        y = T06[1, 3]
        z = T06[2, 3]

        # Extraer orientación R_3x3 y convertir a cuaternión
        R = T06[0:3, 0:3]
        qx, qy, qz, qw = self.rotation_matrix_to_quaternion(R)

        # Imprimir resultados en consola
        self.get_logger().info('\n' + '='*40 +
            f'\nPosición del Efector Final (m):' +
            f'\n  X: {x:.4f}' +
            f'\n  Y: {y:.4f}' +
            f'\n  Z: {z:.4f}' +
            f'\nOrientación (Cuaternión [x, y, z, w]):' +
            f'\n  [{qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f}]' +
            '\n' + '='*40)

def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()