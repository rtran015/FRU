import can
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String, UInt8
from .vesc import Vesc
from rcl_interfaces.msg import ParameterDescriptor


class motor_controller(Node):
    # remember bit conversions are INT16 - Big Endian (AB)
    def __init__(self):
        super().__init__("motor_controller")
        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "ros_topics",
                    [""],
                    ParameterDescriptor(
                        description="List of all topics that need to be convert and sent in the CANbus"
                    ),
                ),
            ],
        )
        # can bus yipie (switch channel name to 'vcan0' for virtual can testing)
        self.bus = can.interface.Bus(
            interface="socketcan", channel="can0", bitrate="500000"
        )
        topic_list: list = self.get_parameter("ros_topics").value
        self.get_logger().debug(f"{topic_list}")

        if not topic_list:
            self.get_logger().error(f"No Topics in Params File -> {topic_list}")

        for topic in topic_list:
            self.create_subscription(
                UInt8,
                topic,
                lambda msg, topic=topic: self.listener_callback(msg, topic),
                10,
            )

    def can_publish(self, arbitration_id, data, is_extended_id) -> None:
        can_msg = can.Message(
            arbitration_id=arbitration_id, data=data, is_extended_id=is_extended_id
        )
        self.bus.send(can_msg)

    def listener_callback(self, msg: UInt8, topic: String):
        print(topic, msg.data)

        # STM32
        if topic == "dt_l_pub":
            self.can_publish(
                Vesc.id_conversion(1, 0), [msg.data], True
            )

        elif topic == "dt_r_pub":
            self.can_publish(
                Vesc.id_conversion(2, 0), [msg.data], True
            )
           


def main(args=None):
    rclpy.init(args=args)
    node = motor_controller()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
