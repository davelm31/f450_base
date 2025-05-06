import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from f450_base.function import *

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        # Publicador de velocidades de motor
        self.publisher = self.create_publisher(Actuators, '/gazebo/command/motor_speed', 10)
        self.timer = self.create_timer(0.01, self.publish_motor_speeds)

        # Suscriptor al IMU
        self.subscription = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        # Inicializar aceleraciones actuales
        self.ddx = 0.0
        self.ddy = 0.0
        self.ddz = 0.0

        self.last_z = 0.0
        self.dz = 0.0
        self.dy = 0.0
        self.dx = 0.0
        # Inicializar aceleraciones actuales
        self.z = 0.0
        self.y = 0.0
        self.x = 0.0

        # Estado actual de p, q, r
        self.p = 0.0
        self.q = 0.0
        self.r = 0.0

        # Inicialización de las variables de roll, pitch y yaw
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.last_time = 0.0

    def publish_motor_speeds(self):

        R = I_R_D(self.roll,self.pitch,self.yaw)
        # Llamar a var2vasi para obtener las derivadas de roll, pitch, yaw
        euler_dot = var2vasi(self.roll, self.pitch, self.yaw, self.p, self.q, self.r)

        # Integrar las derivadas para obtener los nuevos valores de roll, pitch, yaw
        dt = 0.01  
        self.roll += euler_dot[0][0] * dt
        self.pitch += euler_dot[1][0] * dt
        self.yaw += euler_dot[2][0] * dt

        #controlador altura
        kp_z, kd_z, = 0.5, 0.1
        zd, dzd, ddzd = 1, 0, 0

        ddzc = kp_z*(zd - self.z) + kd_z *(dzd - self.dz) + ddzd
        u1 = (ddzc + 9.81)/R[2,2]

        #fuerza de empuje
        u1 = (u1*1.563)/8.54858e-06
        print(self.z)
        
        
        ux = 0
        uy = 0
        uz = 0

        omega= effort2velocityw(u1, ux, uy, uz)

        # Publicar velocidades
        msg = Actuators()
        msg.velocity = omega.tolist()
        self.publisher.publish(msg)
        self.get_logger().info(f"Publicando velocidades: {msg.velocity}")

    def odom_callback(self, msg: Odometry):
        self.z = msg.pose.pose.position.z
        # Obtener tiempo actual y delta t
        current_time = self.get_clock().now().nanoseconds / 1e9  # en segundos
        dt = current_time - self.last_time
        if dt == 0:  # evitar división por cero
            return

        # Primera derivada: velocidad (dz)
        self.dz = (self.z - self.last_z) / 0.01

        # Actualizar para siguiente iteración
        self.last_z = self.z
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
