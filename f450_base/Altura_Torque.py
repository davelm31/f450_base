import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators
from sensor_msgs.msg import Imu
from f450_base.function import *

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        # Publicador de velocidades de motor
        self.publisher = self.create_publisher(Actuators, '/gazebo/command/motor_speed', 50)
        self.timer = self.create_timer(1.0, self.publish_motor_speeds)

        # Suscriptor al IMU
        self.subscription = self.create_subscription(Imu, '/f450b/imu', self.imu_callback, 50)

        # Inicializar aceleraciones actuales
        self.ddx = 0.0
        self.ddy = 0.0
        self.ddz = 0.0
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

    def publish_motor_speeds(self):

        R = I_R_D(self.roll,self.pitch,self.yaw)
        # Llamar a var2vasi para obtener las derivadas de roll, pitch, yaw
        euler_dot = var2vasi(self.roll, self.pitch, self.yaw, self.p, self.q, self.r)

        # Integrar las derivadas para obtener los nuevos valores de roll, pitch, yaw
        dt = 0.001  
        self.roll += euler_dot[0][0] * dt
        self.pitch += euler_dot[1][0] * dt
        self.yaw += euler_dot[2][0] * dt

        # Aceleración del cuerpo respecto al mundo
        acc_body = np.array([self.ddx, self.ddy, self.ddz])
        acc_world = R @ acc_body

        # Integrar para obtener velocidad
        self.dz += acc_world[2] * dt
        # Integrar para obtener posición
        self.z += self.dz * dt

        #controlador altura
        kp_z, kd_z, = 1, 0.01
        zd, dzd, ddzd = 1, 0, 0

        ddzc = kp_z*(zd - self.z) + kd_z *(dzd - self.dz) + ddzd
        u1 = (ddzc + 9.81)/R[2,2]

        #fuerza de empuje
        u1 = (u1*1.563)/8.54858e-06
        print(self.z)
        
        #control x y
        #kp_x, kd_x, = 1, 0.1
        #kp_y, kd_y, = 1, 0.1
        #xd, dxd, ddxd = 0, 0, 0
        #yd, dyd, ddyd = 0, 0, 0

        #ddxc = kp_x*(xd - self.x) + kd_x *(dxd - self.dx) + ddxd
        #ddyc = kp_y*(yd - self.y) + kd_y *(dyd - self.dy) + ddyd

        #valor bx y by
        #b_c_x, b_c_y = p_posicion(ddxc, ddyc, u1)
        #control roll pithc yaw
        #kp12, kp22 = 1, 1
        #db_c_x = kp12 *(b_c_x - R[0,2])
        #db_c_y = kp22 *(b_c_y - R[1,2])


        #ad
        #kpr = 1
        #rolld = 0
        #rc = kpr*(rolld - self.roll)
        #pc,qc = p_velocity(R, db_c_x, db_c_y)

        #Inercias
        Ixx,Ixy,Ixz = 2.277e7, 0, 0
        Iyx,Iyy,Iyz = 0, 2.243e7, 0
        Izx,Izy,Izz = 0, 0, 4.38e7

        #controlador velocidades
        kp_v1, kp_v2, kp_v3 = 0.1, 0.1, 0.1
        pc, qc, rc = 0, 0, 0 

        pdot = kp_v1*(pc - self.p)
        qdot = kp_v2*(qc - self.q)
        rdot = kp_v3*(rc - self.r)

        ux,uy,uz = p_torque_values(Ixx, Iyy, Izz, Ixy, Ixz, Iyz,Izx,Izy,Iyx, 
                    pdot, qdot, rdot, 
                    self.p, self.q, self.r)
        
        ux = (ux*Ixx)/(8.54858e-06*0.1595)
        uy = (uy*Iyy)/(8.54858e-06*0.1595)
        uz = (uz*Izz)/(0.016)

        omega= effort2velocityw(u1, ux, uy, uz)

        # Publicar velocidades
        msg = Actuators()
        msg.velocity = omega.tolist()
        self.publisher.publish(msg)
        self.get_logger().info(f"Publicando velocidades: {msg.velocity}")

    def imu_callback(self, msg: Imu):

        # Leer aceleración lineal y velocida angular
        self.ddx = msg.linear_acceleration.x
        self.ddy = msg.linear_acceleration.y
        self.ddz = msg.linear_acceleration.z

        # Actualizar p, q, r con valores del IMU
        self.p = msg.angular_velocity.x
        self.q = msg.angular_velocity.y
        self.r = msg.angular_velocity.z

        # Mostrar aceleraciones
        #self.get_logger().info(
         #   f'IMU -> Aceleración lineal: x={self.ddx:.2f}, y={self.ddy:.2f}, z={self.ddz:.2f}'
        #)

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
