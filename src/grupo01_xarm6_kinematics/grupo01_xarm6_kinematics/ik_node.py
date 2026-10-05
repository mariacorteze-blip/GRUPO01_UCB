#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import numpy as np
import math

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('ik_node')

        # Nombres de las articulaciones del xArm6
        self.joint_names = [
            'joint1', 'joint2', 'joint3', 
            'joint4', 'joint5', 'joint6'
        ]

        # Parámetros del algoritmo iterativo (Sección VII de la guía)
        self.alpha = 0.5            # Factor de actualización / tasa de aprendizaje
        self.max_iter = 200         # Número máximo de iteraciones
        self.tolerance = 1e-3       # Tolerancia de error ep (1 mm)
        self.delta_q = 1e-6         # Paso numérico para derivadas finitas del Jacobiano

        # Límites articulares en radianes (xArm6 típico, ajustar si difiere en tu URDF)
        self.joint_limits = [
            (-math.pi, math.pi),               # joint1
            (-2.18, 2.18),                     # joint2
            (-3.92, 0.17),                     # joint3
            (-math.pi, math.pi),               # joint4
            (-1.69, 3.14),                     # joint5
            (-math.pi, math.pi)                # joint6
        ]

        # Suscripción al tópico /target (Point)
        self.target_sub = self.create_subscription(
            Point,
            '/target',
            self.target_callback,
            10
        )

        # Publicador hacia /joint_states
        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.get_logger().info('Nodo de Cinemática Inversa (IK) por Jacobiano iniciado.')
        self.get_logger().info('Esperando coordenadas en /target (geometry_msgs/msg/Point)...')

    def dh_matrix(self, theta, d, a, alpha):
        """Matriz de transformación D-H Estándar A_i."""
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

    def forward_kinematics_position(self, q):
        """Calcula únicamente la posición [x, y, z] a partir del vector de ángulos q."""
        dh_params = [
            (q[0],                       0.267,   0.0,     math.radians(-90)),
            (q[1] - math.radians(79.35), 0.0,     0.28949, math.radians(0)),
            (q[2] + math.radians(79.35), 0.0,     0.0775,  math.radians(-90)),
            (q[3],                       0.3425,  0.0,     math.radians(90)),
            (q[4],                       0.0,     0.076,   math.radians(-90)),
            (q[5],                       0.097,   0.0,     math.radians(0))
        ]

        T06 = np.identity(4, dtype=np.float64)
        for theta, d, a, alpha in dh_params:
            A_i = self.dh_matrix(theta, d, a, alpha)
            T06 = np.dot(T06, A_i)

        return T06[0:3, 3] # Retorna [x, y, z]

    def compute_positional_jacobian(self, q):
        """Calcula el Jacobiano posicional Jv (3x6) mediante diferencias finitas."""
        Jv = np.zeros((3, 6), dtype=np.float64)
        p_base = self.forward_kinematics_position(q)

        for i in range(6):
            q_inc = np.copy(q)
            q_inc[i] += self.delta_q
            p_inc = self.forward_kinematics_position(q_inc)
            Jv[:, i] = (p_inc - p_base) / self.delta_q

        return Jv

    def clamp_joint_limits(self, q):
        """Mantiene los ángulos calculados dentro de los límites articulares."""
        q_clamped = np.copy(q)
        for i in range(6):
            q_min, q_max = self.joint_limits[i]
            q_clamped[i] = np.clip(q_clamped[i], q_min, q_max)
        return q_clamped

    def target_callback(self, msg: Point):
        """Procesa el punto objetivo cartesiano enviado a /target."""
        p_d = np.array([msg.x, msg.y, msg.z], dtype=np.float64)

        # Configuración inicial (semilla q0) - Estado 'home' razonable
        q_current = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
        q_0 = np.copy(q_current)

        converged = False
        final_iterations = 0
        final_error = 0.0

        # Algoritmo numérico iterativo
        for k in range(self.max_iter):
            # 1. Posición actual con FK
            p_k = self.forward_kinematics_position(q_current)

            # 2. Vector de error de posición: e_k = p_d - p(q_k)
            e_k = p_d - p_k
            error_norm = np.linalg.norm(e_k)

            # 3. Verificar criterio de convergencia
            if error_norm < self.tolerance:
                converged = True
                final_iterations = k
                final_error = error_norm
                break

            # 4. Jacobiano posicional en la configuración actual
            J_k = self.compute_positional_jacobian(q_current)

            # 5. Pseudoinversa de Moore-Penrose J+
            J_pinv = np.linalg.pinv(J_k)

            # 6. Actualización articular: q_{k+1} = q_k + alpha * J+ * e_k
            q_next = q_current + self.alpha * np.dot(J_pinv, e_k)

            # Respetar límites articulares
            q_current = self.clamp_joint_limits(q_next)

            final_iterations = k + 1
            final_error = error_norm

        p_final = self.forward_kinematics_position(q_current)

        # Reporte claro en consola (exigido por sección VIII-C del informe)
        self.get_logger().info('\n' + '='*50 +
            f'\n--- RESULTADOS DE CINEMÁTICA INVERSA ---' +
            f'\nObjetivo Recibido (p_d) : [{p_d[0]:.4f}, {p_d[1]:.4f}, {p_d[2]:.4f}] m' +
            f'\nConfiguración Inicial (q0): {np.round(q_0, 4).tolist()}' +
            f'\nEstado de Convergencia  : {"CONVERGIÓ" if converged else "NO CONVERGIÓ"}' +
            f'\nNúmero de Iteraciones   : {final_iterations}' +
            f'\nPosición Final Alcanzada: [{p_final[0]:.4f}, {p_final[1]:.4f}, {p_final[2]:.4f}] m' +
            f'\nError Final (ep)        : {final_error:.6f} m' +
            f'\nSolución Articular (q*) : {np.round(q_current, 4).tolist()} rad' +
            '\n' + '='*50)

        # Publicar la solución en /joint_states si convergió o la mejor aproximación
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = q_current.tolist()
        self.joint_pub.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()