import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ParkingControl(Node):
    def __init__(self):
        super().__init__('parking_control_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.3  # Move forward at 0.3 m/s
        self.publisher.publish(twist)

    def turn_left(self):
        twist = Twist()
        twist.angular.z = 0.5  # Turn left at 0.5 rad/s
        self.publisher.publish(twist)

    def turn_right(self):
        twist = Twist()
        twist.angular.z = -0.5  # Turn right at -0.5 rad/s
        self.publisher.publish(twist)

    def stop(self):
        twist = Twist()
        self.publisher.publish(twist)

def main():
    rclpy.init()
    node = ParkingControl()

    # Example usage:
    node.move_forward()
    rclpy.spin_once(node, timeout_sec=5)
    node.turn_left()
    rclpy.spin_once(node, timeout_sec=3)
    node.stop()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
