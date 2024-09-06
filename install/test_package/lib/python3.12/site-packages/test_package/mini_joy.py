import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String, UInt8, Bool
import time

class MiniJoy(Node):
    def __init__(self):
        super().__init__('mini_joy')

        self.dt_l_publisher_ = self.create_publisher(UInt8, 'dt_left', 10)
        self.dt_r_publisher_ = self.create_publisher(UInt8, 'dt_right', 10)

        self.subscription = self.create_subscription(
			Joy,
			'joy',
			self.listener_callback,
			10)

        self.speed_limit = 10
        self.DEADBAND = 0.05

    def listener_callback(self, msg: Joy):
        uint8 = UInt8()
        # Left Stick Maps - Left Drive Train
        if msg.axes[1] > self.DEADBAND: # L Stick Up 
            uint8.data = int((msg.axes[1] * self.speed_limit)) # 1-100 to indicate forward motion
            self.dt_l_publisher_.publish(uint8)

        elif msg.axes[1] < -self.DEADBAND: # L Stick Down 
            uint8.data = int((abs(msg.axes[1]) * self.speed_limit) + 100) # 101-200 so backward movement
            #bug where if its too small it will round down and crank speed to 100%
            if uint8.data == 100:
                uint8.data = 0
            self.dt_l_publisher_.publish(uint8)

        else:
            uint8.data = 0 # deadband resets it to neutral
            self.dt_l_publisher_.publish(uint8)

        # Right Stick Maps - Right Drive Train
        if msg.axes[3] > self.DEADBAND: # R Stick Up
            uint8.data = int((msg.axes[3] * self.speed_limit)) # add 100 to indicate forward motion and not include 100
            self.dt_r_publisher_.publish(uint8)

        elif msg.axes[3] < -self.DEADBAND: # R Stick Down
            uint8.data = int((abs(msg.axes[3]) * self.speed_limit) + 100) # 101-200 so backwards
            #bug where if its too small it will round down and crank speed to 100%
            if uint8.data == 100:
                uint8.data = 0
            self.dt_r_publisher_.publish(uint8)
            
        else:
            uint8.data = 0 # deadband resets it to neutral
            self.dt_r_publisher_.publish(uint8)

def main(args=None):
    rclpy.init(args=args)

    mini_joy = MiniJoy()

    rclpy.spin(mini_joy)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mini_joy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()