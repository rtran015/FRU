import rclpy
import time
from rclpy.node import Node, Publisher
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8
from can import Message, Bus
from rcl_interfaces.msg import ParameterDescriptor


class JoyPub(Node):
    def __init__(self):
        super().__init__("controller_publisher")
        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "deadband",
                    0.05,
                    ParameterDescriptor(
                        description="Controls joystick senesitivity to start reading data"
                    ),
                ),
                (
                    "speed_limit",
                    1,
                    ParameterDescriptor(
                        description="Controls Drive Train max speed (Percentage)"
                    ),
                ),
            ],
        )

        self.dt_l_pub = self.create_publisher(UInt8, "dt_l_pub", 10)
        self.dt_r_pub = self.create_publisher(UInt8, "dt_r_pub", 10)
        self.subscription = self.create_subscription(
            Joy, "joy", self.listener_callback, 10
        )

        self.deadband = self.get_parameter("deadband").value
        self.speed_limit = self.get_parameter("speed_limit").value

    def pubDelay(self, publisher: Publisher, data, delay: float):
        publisher.publish(data)
        time.sleep(delay)

    def listener_callback(self, msg: Joy):
        uint8 = UInt8()

        # Left Stick Maps - Left Drive Train
        # Don't Touch this, it works
        # 0-100 is forward, 100-200 is backward
        if msg.axes[1] > self.deadband:
            uint8.data = int(msg.axes[1] * self.speed_limit)
        elif msg.axes[1] < -self.deadband:
            uint8.data = int(abs(msg.axes[1]) * self.speed_limit + 100)
            uint8.data = 0 if uint8.data == 100 else uint8.data
        else:
            uint8.data = 0  # deadband resets it to neutral
        self.dt_l_pub.publish(uint8)

        # Right Stick Maps - Right Drive Train
        # Don't Touch this either, it works
        # 0-100 is forward, 100-200 is backward
        if msg.axes[4] > self.deadband:
            uint8.data = int((msg.axes[4] * self.speed_limit))
        elif msg.axes[4] < -self.deadband:
            uint8.data = int((abs(msg.axes[4]) * self.speed_limit) + 100)
            uint8.data = 0 if uint8.data == 100 else uint8.data
        else:
            uint8.data = 0  # deadband resets it to neutral
        self.dt_r_pub.publish(uint8)

def main():
    print("Controller On")
    rclpy.init(args=None)

    node = JoyPub()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
