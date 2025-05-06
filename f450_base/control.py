import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class AltitudeController(Node):
    def __init__(self):
        super().__init__('altitude_controller')

        self.goal_z = 1.0           # Altura deseada en metros
        self.Kp = 1              # Ganancia proporcional
        self.max_vel = 0.5          # Velocidad máxima en m/s

        self.current_z = 0.0        # Altura actual

        self.publisher = self.create_publisher(Twist, '/gazebo/command/twist', 10)
        self.subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.current_z = msg.pose.pose.position.z

    def control_loop(self):
        # Calcula el error en altura
        error_z = self.goal_z - self.current_z
        vz = self.Kp * error_z

        # Limita la velocidad para evitar oscilaciones
        vz = max(min(vz, self.max_vel), -self.max_vel)

        # Construye y publica el comando Twist
        msg = Twist()
        msg.linear.z = vz

        self.publisher.publish(msg)

        self.get_logger().info(f'Altura actual: {self.current_z:.2f} m | Velocidad z: {vz:.2f} m/s')

def main(args=None):
    rclpy.init(args=args)
    node = AltitudeController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
