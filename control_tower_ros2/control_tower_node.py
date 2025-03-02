import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from control_tower_ros2.ackermann_mode import ackermann_drive
from control_tower_ros2.differential_mode_mode import differential_drive
from control_tower_ros2.directional_mode_mode import directional_drive
        
class control_tower_node(Node):
    def __init__(self):
        super().__init__('control_tower_node')

        # Create a publisher for the Twist message on the 'cmd_vel' topic.
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)

        # Publisher for the switch states using an array of integers
        self.switch_publisher_ = self.create_publisher(Int32MultiArray, 'switch_states', 1)

        # Set up a timer to call update_callback periodically (e.g., every 0.1 seconds)
        self.timer = self.create_timer(0.01, self.update_callback)

        # Stick
        self.ry = 0  # Vroom vroom (CH3)
        self.rx = 0  # steering (CH1)
        self.ly = 0  # Throttle (CH2)
        self.lx = 0  # we dont know (CH4)

        # Switch
        self.sw_a = 0
        self.sw_b = 0 #broken
        self.sw_c = 0
        self.sw_d = 0

        # Create subscriptions
        self.sub_ch1 = self.create_subscription(Int32,'ch1', self.callback_1, 1)
        self.sub_ch2 = self.create_subscription(Int32,'ch2', self.callback_2, 1)
        self.sub_ch3 = self.create_subscription(Int32,'ch3', self.callback_3, 1)
        self.sub_ch4 = self.create_subscription(Int32,'ch4', self.callback_4, 1)

        self.sub_ch5 = self.create_subscription(Int32,'ch5', self.callback_5, 1)
        self.sub_ch6 = self.create_subscription(Int32,'ch6', self.callback_6, 1)
        self.sub_ch7 = self.create_subscription(Int32,'ch7', self.callback_7, 1)
        self.sub_ch8 = self.create_subscription(Int32,'ch8', self.callback_8, 1)

    def map_rc(self, value, out_min=-10.0, out_max=10.0):
        return (round((value - 1000) * (out_max - out_min) / 1000 + out_min, 1))

    def map_sw(self, value):
        if value == 1000:
            return 0
        elif value == 1500:
            return 1
        else:
            return 2
    
    # Define separate callback functions for each channel
    def callback_1(self, msg): self.rx = msg.data
    def callback_2(self, msg): self.ly = msg.data
    def callback_3(self, msg): self.ry = msg.data
    def callback_4(self, msg): self.lx = msg.data

    def callback_5(self, msg): self.sw_a = msg.data
    def callback_6(self, msg): self.sw_b = msg.data
    def callback_7(self, msg): self.sw_c = msg.data
    def callback_8(self, msg): self.sw_d = msg.data
 
    def update_callback(self):
        # Map the IBUS data to a Twist message.
        twist_msg = Twist()

        if self.sw_c == 0:
            pass

        elif self.sw_c == 1:
            pass

        elif self.sw_c == 2:
            pass


"""     
        twist_msg.linear.x = self.map_rc(self.ly)
        twist_msg.angular.z = self.map_rc(self.lx,-3.14,3.14)
        #twist_msg.linear.y = self.angular_y
        #twist_msg.angular.x = self.

        # Publish the Twist message
        self.publisher_.publish(twist_msg)
        self.get_logger().info(f"Published Twist: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}")

        #Publish the Switch state
        sw_msg = Int32MultiArray()
        sw_msg.data = [
            self.map_sw(self.sw_a),
            self.map_sw(self.sw_b),
            self.map_sw(self.sw_c),
            self.map_sw(self.sw_d)
        ]
        self.switch_publisher_.publish(sw_msg)
        self.get_logger().info(f"Published Switch States: {sw_msg.data}") """

def main(args=None):
    rclpy.init(args=args)
    node = control_tower_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
