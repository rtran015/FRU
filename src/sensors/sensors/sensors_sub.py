import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray, UInt8, String, Int32, Int16, Int32MultiArray
#ignore can import error if it's there, it works if you installed python-can
import can

class SensorSub(Node):
    def __init__(self):
        super().__init__('sensors_sub')

        filters=[
                {"can_id": 0x00000900, "can_mask": 0x1FFFFF00, "extended": True},
                {"can_id": 0x00000E00, "can_mask": 0x1FFFFF00, "extended": True},
                {"can_id": 0x00000F00, "can_mask": 0x1FFFFF00, "extended": True},
                {"can_id": 0x00001000, "can_mask": 0x1FFFFF00, "extended": True},
                {"can_id": 0x00001B00, "can_mask": 0x1FFFFF00, "extended": True}
        ]
        # Standard CAN
        self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate='500000', can_filters=filters)
        
        self.vesc1_publisher_ = self.create_publisher(Int32MultiArray, 'vesc_pub', 10) 
        self.status_timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        self.bus.flush_tx_buffer()
        msg = self.bus.recv()
        # self.get_logger().info(f'Id: {msg.arbitration_id}, Time: {msg.timestamp}, Data: {msg.data}')
        match msg.arbitration_id >> 8:
            case 9:
                #self.get_logger().info('rpm')
                d = int.from_bytes(msg.data[0:4] , byteorder='big', signed=True)
                num = Int32MultiArray()

                num.data = [msg.arbitration_id, 0, d]
                self.vesc1_publisher_.publish(num)

                #self.get_logger().info('current')
                d = int.from_bytes(msg.data[4:6] , byteorder='big', signed=True)
                num.data = [msg.arbitration_id, 1, d]
                self.vesc1_publisher_.publish(num)
                
                #self.get_logger().info('duty')
                d = int.from_bytes(msg.data[6:8] , byteorder='big', signed=True)
                num.data = [msg.arbitration_id, 2, d]
                self.vesc1_publisher_.publish(num)


            case 14:
                #self.get_logger().info('amp')
                num = Int32MultiArray()

                d = int.from_bytes(msg.data[0:4] , byteorder='big', signed=True)
                num.data = [msg.arbitration_id, 0, d]
                self.vesc1_publisher_.publish(num)

                d = int.from_bytes(msg.data[4:8] , byteorder='big', signed=True)
                num.data = [msg.arbitration_id, 1, d]
                self.vesc1_publisher_.publish(num)

            case 15:
                #self.get_logger().info('watt')
                num = Int32MultiArray()

                d = int.from_bytes(msg.data[0:4] , byteorder='big', signed=True)
                num.data = [msg.arbitration_id, 0, d]
                self.vesc1_publisher_.publish(num)

                d = int.from_bytes(msg.data[4:8] , byteorder='big', signed=True)
                num.data = [msg.arbitration_id, 1, d]
                self.vesc1_publisher_.publish(num)

            case 16:
                #self.get_logger().info('temp')
                num = Int32MultiArray()

                d = int.from_bytes(msg.data[0:2] , byteorder='big', signed=True)
                num.data = [msg.arbitration_id, 0, d]
                self.vesc1_publisher_.publish(num)               

                d = int.from_bytes(msg.data[2:4] , byteorder='big', signed=True)
                num.data = [msg.arbitration_id, 1, d]
                self.vesc1_publisher_.publish(num)

                d = int.from_bytes(msg.data[4:6] , byteorder='big', signed=True)
                num.data = [msg.arbitration_id, 2, d]
                self.vesc1_publisher_.publish(num)

                d = int.from_bytes(msg.data[6:8] , byteorder='big', signed=True)
                num.data = [msg.arbitration_id, 3, d]
                self.vesc1_publisher_.publish(num)
            
            case 27:
                #self.get_logger().info('speed')
                num = Int32MultiArray()

                d = int.from_bytes(msg.data[0:4] , byteorder='big', signed=True)
                num.data = [msg.arbitration_id, 0, d]
                self.vesc1_publisher_.publish(num)

                d = int.from_bytes(msg.data[4:6] , byteorder='big', signed=True)
                num.data = [msg.arbitration_id, 1, d]
                self.vesc1_publisher_.publish(num)


def main(args=None):
    print("Sensor Sub Active!!!!")
    rclpy.init(args=args)
    node = SensorSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
