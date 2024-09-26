import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8, String
import time
#ignore can import error if it's there, it works if you installed python-can
import can


class MainDriveAuto(Node):
    
    def __init__(self):
        super().__init__('main_drive_auto')

        #joy listener
        self.subscription = self.create_subscription(
			Joy,
			'joy',
			self.listener_callback,
			10)

        # create can bus link, right now is linked to virtual vcan 0, most likely
        # will be can0 when on the bot
        self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate='500000')

        self.in_action = False
    
    # Converts Controller Speed to byte array (decimal form)
    # Alg: signal -> percentage * 1000 (UInt16) -> Hexadecimal Byte Form -> Decimal Byte Form 
    # Ex. 200 -> 50% -> 50,000 = [80, 200]
    def signal_conversion(self, msg_data: int, bytes_range: int, frequency_floor: int) -> list[int]:
        data: int = msg_data 
        temp_data: list[int] = []

        #make sure the direction is correct
        if data > 100:
            #increment for 2's comp
            c = 1
            # Forward msg correction:
            data -= 100
            # covert controller signal to proper range (1000-100000)
            data *= frequency_floor

            #convert to byte array but also 2's compliment to reverse motor
            for i in range(bytes_range - 1, -1, -1):
                temp_data.append(255 - ((data >> (8*i)) & 0xff))

            for i in range(len(temp_data) - 1, - 1, -1):
                temp_data[i] += c 
                if temp_data[i] > 255:
                    temp_data[i] = 0
                else:
                    c = 0
                    break
        else:
            # covert controller signal to proper range (1000-100000)
            data *= frequency_floor
    
            # convert signal to byte array
            for i in range(bytes_range - 1, -1, -1):
                temp_data.append((data >> (8*i)) & 0xff)
        
        return temp_data 

    def can_publish(self, arbitration_id, data, is_extended_id) -> None:
        can_msg = can.Message(
                arbitration_id = arbitration_id,
                data = data, 
                is_extended_id = is_extended_id
                ) 
        self.bus.send(can_msg)

    #Duty 0 is stop, 1-100 is forward, 101-200 is backwards. Time is in ms
    def drive_for_time(self, left_duty, right_duty, time):
        self.in_action = True
        left_data = self.signal_conversion(left_duty, 4, 1000)
        right_data = self.signal_conversion(right_duty, 4, 1000)
        start_time = time.time()
        while((time.time() - start_time) < (time / 1000)):
            self.can_publish(15, left_data, True)
            self.can_publish(16, left_data, True)
            self.can_publish(17, right_data, True)
            self.can_publish(18, right_data, True)

            # hardstop at 3 seconds for safety since we are testing
            if((time.time() - start_time) > 3000):
                break
        self.in_action = False

    def listener_callback(self, msg):
        # X Button
        if msg.buttons[3] == 1 and not self.in_action:
            #we are philosphers eating rice and the robot is the chopstick
            self.drive_for_time(10, 10, 2000)

                


def main(args=None):
    print("Main Drive Auto Mode")
    rclpy.init(args=args)
    node = MainDriveAuto()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
