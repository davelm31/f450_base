import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time

class ImuListener(Node):
    def __init__(self):
        super().__init__('imu_listener')

        self.subscription = self.create_subscription(
            Imu,'/f450b/imu',self.imu_callback,10)

        # Variables de estado para integración
        self.prev_time = self.get_clock().now()
        self.vel = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}

    def imu_callback(self, msg):
        # Obtener aceleraciones
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z - 9.8  # Corregimos la gravedad

        # Calcular delta t
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9  # en segundos
        self.prev_time = now

        # Integración simple (Euler)
        self.vel['x'] += ax * dt
        self.vel['y'] += ay * dt
        self.vel['z'] += az * dt

        self.pos['x'] += self.vel['x'] * dt
        self.pos['y'] += self.vel['y'] * dt
        self.pos['z'] += self.vel['z'] * dt

        # Imprimir posición estimada
        self.get_logger().info(
            f'Aceleración corregida -> x: {ax:.2f}, y: {ay:.2f}, z: {az:.2f} | '
            f'Posición estimada -> x: {self.pos["x"]:.2f}, y: {self.pos["y"]:.2f}, z: {self.pos["z"]:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = ImuListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
