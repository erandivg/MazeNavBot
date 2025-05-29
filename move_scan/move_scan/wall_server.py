import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from service_pkg.srv import FindWall  # ✅ Tu servicio personalizado

import math

class Service(Node):

    def __init__(self):
        super().__init__('find_wall_service')

        # ✅ Servicio personalizado
        self.srv = self.create_service(FindWall, '/find_wall', self.service_callback)

        # Suscripción al láser
        self.sub_laser = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Publicador de velocidades
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.vel = TwistStamped()

        # Estados internos
        self.wall_found = False
        self.find_wall_active = False
        self.min_distance = float('inf')

        # Timer para lógica periódica
        self.timer = self.create_timer(0.1, self.timer_callback)

    def laser_callback(self, msg):
        self.min_distance = min(msg.ranges)
        self.laser_right = msg.ranges[90]
        self.laser_forward = msg.ranges[0]

        # Guardar como string los primeros dígitos
        self.first_two_forward = str(round(self.laser_forward, 2))[:3]
        self.first_two_min_distance = str(round(self.min_distance, 2))[:3]

    def service_callback(self, request, response):
        self.get_logger().info("Servicio recibido: iniciar búsqueda del muro.")
        self.find_wall_active = True
        self.wall_found = False

        response.wallfound = True  
        return response

    def timer_callback(self):
        if not self.find_wall_active or self.wall_found:
            return

        try:
            forward = float(self.first_two_forward)
            min_dist = float(self.first_two_min_distance)
        except Exception as e:
            self.get_logger().warn(f"Error parsing laser distances: {e}")
            return

        wall_distance = self.laser_right - min_dist
        if math.isinf(wall_distance) or math.isinf(self.laser_right):
            wall_distance = 10.0
            self.get_logger().info(f"wall_distance se cambió: {wall_distance}")

        if wall_distance > 0.5:
            self.vel.twist.linear.x = 0.05
            self.vel.twist.angular.z = 0.25
            self.get_logger().info(f"Moviendo... forward: {forward}, min: {min_dist}")
            self.get_logger().info(f"wall_distance: {wall_distance}")
        else:
            self.wall_found = True
            self.find_wall_active = False
            self.get_logger().info("¡Muro encontrado! Deteniendo robot.")
            self.vel.twist.angular.z = 0.0
            self.vel.twist.linear.x = 0.0

        self.publisher_.publish(self.vel)

def main(args=None):
    rclpy.init(args=args)
    service = Service()
    executor = MultiThreadedExecutor()
    executor.add_node(service)

    try:
        executor.spin()
    finally:
        service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
