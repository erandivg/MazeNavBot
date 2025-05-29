import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

import math

class WallFollowing(Node):

    def __init__(self):
        super().__init__('wall_following')

        # create the publisher object
        self.pub_vel = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        # create the subscriber object
        self.sub_laser = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        
        self.timer_period = 0.1

        self.laser_forward = 0
        self.laser_left = 0
        self.laser_right = 0
        self.stop = False
        # create a Twist message
        self.vel = TwistStamped()
        self.timer = self.create_timer(self.timer_period, self.move_robot)

    def laser_callback(self, msg):
        num_points = len(msg.ranges)
        forward_index = 0
        right_index = int(num_points * 0.25)
        left_index = int(num_points * 0.75)

        # Función segura para limpiar lecturas inválidas
        def safe_read(index):
            value = msg.ranges[index]
            if math.isinf(value) or math.isnan(value):
                return 10.0  # valor arbitrario alto si no hay obstáculo
            return value

        self.laser_forward = safe_read(forward_index)
        self.laser_right = safe_read(right_index)
        self.laser_left = safe_read(left_index)

        #print the log info in the terminal
        #self.get_logger().info('I receive enfrente: "%s"' % str(self.laser_forward))
        #self.get_logger().info('I receive izquierda: "%s"' % str(self.laser_left))   
        #self.get_logger().info('I receive derecha: "%s"' % str(self.laser_right))
        #self.get_logger().info(f'Angle Min: {msg.angle_min}, Angle Max: {msg.angle_max}')
        #self.get_logger().info(f'Número de puntos del láser: {num_points}')


    def move_robot(self):

        if not self.stop:

            self.get_logger().info(f'Forward: {self.laser_forward}')
            self.get_logger().info('Right "%s"' % str(self.laser_right))
            self.get_logger().info('Left "%s"' % str(self.laser_left))


            if self.laser_forward > 1.2:
            
                if self.laser_right < 0.6:
                    self.vel.twist.angular.z = -0.1
                    self.vel.twist.linear.x = 0.2
                elif self.laser_left < 0.6:
                    self.vel.twist.angular.z = 0.1
                    self.vel.twist.linear.x = 0.2
                else:
                    self.vel.twist.linear.x= 0.2
                    self.vel.twist.angular.z = 0.0

            if self.laser_forward > 9.5 and self.laser_left > 9.5 and self.laser_right > 9.5:
                self.vel.twist.angular.z = 0.0
                self.vel.twist.linear.x = 0.0
                self.get_logger().info('Stop')
                self.stop = True
                

            if self.laser_forward <= 1.3:

                if self.laser_right > self.laser_left:
                    self.vel.twist.angular.z = 0.2
                elif self.laser_right < self.laser_left:
                    self.vel.twist.angular.z = -0.2
                
                self.vel.twist.linear.x = 0.12


            #self.vel.angular.z = -0.2
            self.pub_vel.publish(self.vel)
  
def main(args=None):
    rclpy.init(args=args)
    wall_following = WallFollowing()
    rclpy.spin(wall_following)
    wall_following.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
