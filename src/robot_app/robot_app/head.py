import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import numpy as np
from std_msgs.msg import UInt8, Bool
from sensor_msgs.msg import LaserScan

# Импорты
from .parking import Parking_Handler

# Глобальные переменные
enable_pedestrian_node = False

signs = {
    "red_light": 0,
    "yellow_light": 1,
    "green_light": 2,
    "intersection": 3,
    "right": 4,
    "left": 5,
    "construction": 6,
    "parking": 7,
    "pedestrian": 8,
    "tunnel": 9
}

states = {
    "disable": 0,
    "stop": 1,
    "forward": 2,
    "left": 3,
    "right": 4
}


class Pedestrian(Node):
    def __init__(self):
        super().__init__('pedestrian')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_scan_callback,
            10
        )
        self.waiting = False
        self.pub = self.create_publisher(Bool, '/knight_on_the_road', 10)

    def laser_scan_callback(self, msg):
        if enable_pedestrian_node:
            self.compute(msg)

    def compute(self, msg):
        # [164:197]
        center_point = len(msg.ranges)
        view1 = np.array(msg.ranges[center_point - 18: center_point - 1])
        view2 = np.array(msg.ranges[:10])
        view = np.concatenate((view1, view2), axis=0)
        pedestrian_msg = Bool()
        if np.any(view < 0.42):
            self.waiting = True
            pedestrian_msg.data = True 
        elif self.waiting:
            pedestrian_msg.data = False 
        if self.waiting: 
            self.pub.publish(pedestrian_msg)


class Head(Node):
    def __init__(self):
        super().__init__('head')
        self.state_publisher = self.create_publisher(UInt8, '/state', 10)
        self.detection_sub = self.create_subscription(UInt8, '/sign_detection', self.detection_callback, 10)
        self.pedestrian_sub = self.create_subscription(Bool, '/knight_on_the_road', self.pedestrian_callback, 10)
        self.current_state = states["stop"]
        self.parking_handler = Parking_Handler() # Создаем объект Parking_Handler

    def detection_callback(self, msg):
        self.get_logger().info(f"Sign message received: {list(signs.keys())[msg.data]}")
        self.sign_handler(msg.data)

    def pedestrian_callback(self, msg):
        global enable_pedestrian_node
        if msg.data:
            self.current_state = states["stop"]
        else: 
            self.current_state = states["forward"]
            enable_pedestrian_node = False
        self.publish_state()

    def sign_handler(self, sign):
        global enable_pedestrian_node
        if sign == signs["red_light"] or sign == signs["yellow_light"]:
            self.current_state = states["stop"]
        elif sign == signs["green_light"]:
            self.current_state = states["forward"]
        elif sign == signs["intersection"]:
            pass 
        elif sign == signs["right"]:
            self.current_state = states["right"]
        elif sign == signs["left"]:
            self.current_state = states["left"]
        elif sign == signs["construction"]:
            self.current_state = states["right"]
        elif sign == signs["parking"]:
            self.get_logger().info("Parking sign detected, activating Parking Handler...")
            self.parking_handler.is_parking = True  # Активируем обработчик парковки
            # self.current_state = states["stop"]  # Переключаем состояние на STOP
        elif sign == signs["pedestrian"]:
            self.current_state = states["forward"]
            enable_pedestrian_node = True
        elif sign == signs["tunnel"]:
            enable_pedestrian_node = False

        self.get_logger().info(f"State handled: {list(states.keys())[self.current_state]}")
        self.publish_state()

    def publish_state(self):
        state_msg = UInt8()
        state_msg.data = self.current_state
        self.state_publisher.publish(state_msg)
        self.get_logger().info(f"State published: {list(states.keys())[state_msg.data]}\n")


def main(args=None):
    rclpy.init(args=args)
    head = Head()
    pedestrian = Pedestrian()
    
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(head)
    executor.add_node(pedestrian)
    executor.add_node(head.parking_handler) 
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
