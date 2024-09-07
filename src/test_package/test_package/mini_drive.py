import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
import can

class MiniDrivetrain(Node):

    def __init__(self):
        super().__init__('mini_drivetrain')
        
        # Listeners for commands controlling the left and right drivetrain motors
        self.dt_left_sub = self.create_subscription(UInt8, 'dt_left', self.left_update, 10)
        self.dt_right_sub = self.create_subscription(UInt8, 'dt_right', self.right_update, 10)

        # drivetrain periodic
        self.drivetrain_timer = self.create_timer(0.1, self.drivetrain_periodic)

        # create state variables, these keep track of what motors should be running and how fast at the current moment
        self.dt_left_speed = 0
        self.dt_right_speed = 0  

        # can bust initialization
        self.bus = can.interface.Bus(interface='socketcan', channel='vcan0', bitrate='50000')

    # converts the input integer into a list of Hex grouped by every 2 hex digits, stored as integers from 0-255
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

    # function to help format publishing can messages
    def can_publish(self, arbitration_id, data, is_extended_id = False) -> None:
        can_msg = can.Message(
                arbitration_id = arbitration_id,
                data = data, 
                is_extended_id = is_extended_id
                ) 
        self.bus.send(can_msg)

    #updates the states of the left drivetrain motors
    def left_update(self, msg):
        # checks if speed is different then previous message published
        if self.dt_left_speed == msg.data:     
            return None
        self.dt_left_speed = msg.data


    #updates the states of the right drivetrains motors
    def right_update(self, msg):
        # checks if speed is different then previous message published
        if self.dt_right_speed == msg.data:
            return None
        self.dt_right_speed = msg.data

    # for publishing the drivetrain can messages to the vescs
    def drivetrain_periodic(self):
        temp_data = self.signal_conversion(self.dt_left_speed, 4, 1000)  # Has to be 4 to work on vesc
        # can message for right and left motor
        self.can_publish(15, temp_data, True)
        self.can_publish(16, temp_data, True) 

        # converts controller signal to bytes array
        temp_data = self.signal_conversion(self.dt_right_speed, 4, 1000)  # Has to be 4 to work on vesc
        self.can_publish(17, temp_data, True)
        self.can_publish(18, temp_data, True)

def main(args=None):
    print("Drive Live")
    rclpy.init(args=args)
    node = MiniDrivetrain()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()