# from time import sleep
# import os

# import rclpy
# from rclpy.node import Node

# from std_msgs.msg import Bool, String, Float64, Int8, UInt8
# from robot_rotate_interface.msg import Rotate
# from sensor_msgs.msg import LaserScan, Image
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry

# from cv_bridge import CvBridge

# import cv2
# import numpy as np

# from sensor_msgs.msg import LaserScan


# class Parking_Handler(Node):
#     """ Проезд парковки.
#     При приближении к знаку переключает режим следования по полосе на 
#     режим следования по двум желтым полосам. После доезда до парковочного места с помощью лидара 
#     определяет, где стоит машина, и поворачивает на парковочное место в противоположную сторону.
#     Для выезда с парковочного места сдает назад, после чего вновь включается движение по двум желтым полосам до поворота.
#     """

#     def __init__(self):
#         super().__init__('parking_handler_node')

#         self.get_logger().info("loadParKING!!!!!!!!!!!!!!!!!!!!!!!!!!!")

#         # Publishers
#         self.enable_following_pub = self.create_publisher(Bool, '/enable_following', 1)
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
#         self.max_vel_pub = self.create_publisher(Float64, '/max_vel', 1)   
#         self.offset_pub = self.create_publisher(Float64, '/offset', 1)
#         self.parking_pub = self.create_publisher(Bool, '/is_parking', 1)
#         self.rotate_pub = self.create_publisher(Rotate, '/rotate', 1)

        
#         # Subscribers
#         self.rotate_done_sub = self.create_subscription(Int8, '/rotate_done', self.set_rotate_done, 1)
#         # self.sign_sub = self.create_subscription(UInt8,'/sign_detection', self.handle_sign, 1)
#         self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.get_distance, 1)
#         self.img_proj_sub = self.create_subscription(Image, '/color/image_projected', self.image_processing, 1)
#         self.sub_odom = self.create_subscription(Odometry, '/odom', self.get_odom, 1)
 
#         self.ID = 7 # Идентификатор ноды

#         self.cv_bridge = CvBridge()

#         self.sind_timer = self.create_timer(0.1, self.handle_sign)

#         # Скорости
#         self.in_speed = 0.50
#         self.parking_speed = 0.35
#         self.out_speed = 0.40
        
#         # Смещения
#         self.in_offset = 60.0 
#         self.out_offset = 30.0 

#         # Углы поворота
#         self.angle_L = 85.0 
#         self.angle_R = -80.0 
#         self.angle_B = 70.0 

#         # Линейные скорости при повороте
#         self.linear_x_L = 0.20 
#         self.linear_x_R = 0.28 
#         self.linear_x_BL = -0.05 
#         self.linear_x_BR = -0.10 

#         # Угловые скорости при повороте
#         self.angular_z_L = 1.0 
#         self.angular_z_R = 1.0 
#         self.angular_z_BL = 0.5 
#         self.angular_z_BR = 0.6 

#         self.is_parking = False # Режим заезда на парковку
#         self.find_parking_place = False # Режим поворота на парковочное место
#         self.exit = False # Режим выезда с перекрестка

#         self.shutdown = False # Нужно ли выключить ноду после завершения поворота
#         self.angle = None     # Угол для поворота на парковочное место 
#         self.linear_x = None  # Линейная скорость при повороте
#         self.angular_z = None # Угловая скорость при повороте

#     # def handle_sign(self, msg):
#     #     self.get_logger().info("not ParKING!!!!!!!!!!!!!!!!!!!!!!!!!!!")
#     #     if int(msg.data) == 7 and not self.exit:
#     #         self.get_logger().info("ParKING!!!!!!!!!!!!!!!!!!!!!!!!!!!")
#     #         self.is_parking = True
#     #         self.max_vel_pub.publish(Float64(data = self.in_speed))
#     #         self.offset_pub.publish(Float64(data = self.in_offset))

#     def handle_sign(self):
#         # self.get_logger().info("not ParKING!!!!!!!!!!!!!!!!!!!!!!!!!!!")
#         if self.is_parking and not self.exit:
#             # self.get_logger().info("ParKING!!!!!!!!!!!!!!!!!!!!!!!!!!!")
#             # self.is_parking = True
#             self.max_vel_pub.publish(Float64(data = self.in_speed))
#             self.offset_pub.publish(Float64(data = self.in_offset))


#     def get_distance(self, msg):
#         # self.get_logger().info("get_distance")
#         # Дожидаемся, пока не доедем до знака
#         if self.is_parking and np.min(msg.ranges[270:360]) < 0.3:
#             self.parking_pub.publish(Bool(data = True))
#             self.max_vel_pub.publish(Float64(data = self.parking_speed))

#         # Определяем в какую сторону нужно повернуть для парковки
#         if self.find_parking_place:

#             self.find_parking_place = False
#             min_id = np.argmin(msg.ranges)

#             # Налево
#             if min_id >= 180:
#                 self.angle = self.angle_L
#                 self.linear_x = self.linear_x_L
#                 self.angular_z = self.angular_z_L

#             # Направо
#             else:
#                 self.angle = self.angle_R
#                 self.linear_x = self.linear_x_R
#                 self.angular_z = self.angular_z_R

#             self.rotate_pub.publish(Rotate(angle = self.angle, linear_x = self.linear_x, angular_z = self.angular_z, id = self.ID))
            
#     def image_processing(self, msg):
#         # self.get_logger().info("image_processing")
#         if self.is_parking:

#             # Считывание изображения и перевод в HSV
#             image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
#             hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

#             # Маска белой линии
#             white_mask = cv2.inRange(hsv_image, (0, 0, 230), (255, 0, 255))
#             white_mask = cv2.blur(white_mask, (3, 3))
#             white_mask[white_mask != 0] = 255

#             contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

#             if not self.find_parking_place:

#                 # Начать поворот на парковочное место
#                 if len(contours) > 4:
#                     self.enable_following_pub.publish(Bool(data = False))
#                     self.cmd_vel_pub.publish(Twist())
#                     self.find_parking_place = True
#                     self.is_parking = False

#     def get_odom(self, msg):
#         # self.get_logger().info("get_odom")
#         if self.exit:
#             pose_y = msg.pose.pose.position.y

#             # Продолжить движение по полосе после выезда с парковки
#             if pose_y >= 3.10:

#                 self.parking_pub.publish(Bool(data = False))
#                 self.enable_following_pub.publish(Bool(data = False))
#                 self.rotate_pub.publish(Rotate(angle = 45.0, linear_x = 0.22, angular_z = 1.0, id = self.ID))

#                 self.max_vel_pub.publish(Float64(data = self.out_speed))
#                 self.offset_pub.publish(Float64(data = self.out_offset))
                
#                 self.shutdown = True 

#     def set_rotate_done(self, msg):
#         # self.get_odom().info("set_rotate_done")
#         if msg.data == self.ID:
#             # Поворот на парковочное место
#             if not self.exit:
#                 self.cmd_vel_pub.publish(Twist())

#                 # Ждем секунду и пока робот полностью остановится
#                 sleep(1.5)

#                 # Запускаем выезд задом
#                 self.angle = self.angle_B * np.sign(self.angle)
#                 self.linear_x = self.linear_x_BL if self.angle > 0 else self.linear_x_BR
#                 self.angular_z = self.angular_z_BL if self.angle > 0 else self.angular_z_BR

#                 self.rotate_pub.publish(Rotate(angle = self.angle, linear_x = self.linear_x, angular_z = self.angular_z, id = self.ID))
                
#                 self.is_parking = False
#                 self.exit = True
                
#             else:
#                 self.enable_following_pub.publish(Bool(data = True))

#                 if self.shutdown:
#                     rclpy.shutdown()


# def main():
#     rclpy.init()
#     # sleep(3.0)

#     node = Parking_Handler()
#     rclpy.spin(node)
    
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np

class ParkingRobot(Node):
    def __init__(self):
        super().__init__('parking_robot')
        
        # Publishers and Subscribers
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber_camera = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.subscriber_lidar = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Utilities
        self.bridge = CvBridge()
        self.twist = Twist()
        self.lidar_data = []  # LiDAR data storage
        self.state = 'FOLLOW_YELLOW_LINES'
        
    def camera_callback(self, msg):
        """
        Обработка изображения с камеры для следования по жёлтым линиям.
        """
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        if self.state == 'FOLLOW_YELLOW_LINES':
            self.follow_yellow_lines(frame)
            self.find_parking_space()
        elif self.state == 'PARK':
            self.park()
        elif self.state == 'EXIT_PARKING':
            self.exit_parking()

    def lidar_callback(self, msg):
        """
        Сохранение данных с LiDAR.
        """
        self.lidar_data = msg.ranges

    def follow_yellow_lines(self, frame):
        """
        Следование за жёлтыми линиями с использованием обработки изображения.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(hsv, (20, 100, 100), (30, 255, 255))  # Диапазон жёлтого цвета

        # Найти контуры жёлтых линий
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Рассчитать центр между жёлтыми линиями
            line_center = self.calculate_center_of_lines(contours, frame.shape)
            frame_center = frame.shape[1] // 2
            error = line_center - frame_center
            
            # Управление движением
            self.twist.linear.x = 0.2  # Скорость вперёд
            self.twist.angular.z = -float(error) / 100  # Коррекция поворота
            self.publisher_cmd_vel.publish(self.twist)
        else:
            self.get_logger().info('Yellow lines not found!')

    def calculate_center_of_lines(self, contours, frame_shape):
        """
        Рассчёт среднего положения жёлтых линий.
        """
        centers = [cv2.moments(cnt)['m10'] / (cv2.moments(cnt)['m00'] + 1e-5) for cnt in contours]
        center_avg = sum(centers) / len(centers)
        return int(center_avg)

    def find_parking_space(self):
        """
        Поиск свободного парковочного места с использованием данных LiDAR.
        """
        if not self.lidar_data:
            return
        
        free_space_threshold = 1.5  # Минимальное расстояние для парковки (метры)
        min_required_depth = 2.0    # Глубина свободного пространства для парковки

        free_indices = []
        for i, distance in enumerate(self.lidar_data):
            if distance > free_space_threshold:
                free_indices.append(i)
            else:
                if len(free_indices) > 0:
                    # Проверка ширины и глубины пространства
                    angle_width = len(free_indices) * 0.01  # Шаг угла 0.01 радиан
                    depth = min(self.lidar_data[free_indices[0]:free_indices[-1]])
                    if angle_width >= free_space_threshold and depth >= min_required_depth:
                        self.get_logger().info('Free parking space found!')
                        self.state = 'PARK'
                        return
                free_indices = []

    def park(self):
        """
        Движение для парковки в свободное пространство.
        """
        self.get_logger().info('Parking...')
        self.twist.linear.x = 0.2  # Двигаться вперёд
        self.twist.angular.z = 0.0
        self.publisher_cmd_vel.publish(self.twist)
        
        # Условный таймер парковки, можно улучшить логикой остановки
        rclpy.sleep(2)  # Пауза на 2 секунды для парковки
        self.state = 'EXIT_PARKING'

    def exit_parking(self):
        """
        Выезд с парковочного места и возвращение на трассу.
        """
        self.get_logger().info('Exiting parking...')
        self.twist.linear.x = -0.2  # Движение назад
        self.twist.angular.z = 0.0
        self.publisher_cmd_vel.publish(self.twist)
        
        rclpy.sleep(2)
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.5  # Поворот для возвращения
        self.publisher_cmd_vel.publish(self.twist)
        
        rclpy.sleep(2)
        self.state = 'FOLLOW_YELLOW_LINES'
        self.get_logger().info('Back on track!')


def main(args=None):
    rclpy.init(args=args)
    node = ParkingRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
