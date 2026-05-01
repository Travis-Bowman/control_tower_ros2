#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import numpy as np
from control_tower_ros2.differentail_drive import DifferentialDrive as diff_drive
from control_tower_ros2.msg import DiffWheelCommands

class control_tower_node(Node):
    

    def __init__(self):
        super().__init__('control_tower_node')
        self.publisher_ = self.create_publisher(Twist, 'rc_cmd_vel', 1)

        self.auto_cmd_vel = Twist()
        self.create_subscription(Twist, 'cmd_vel', self._auto_cmd_vel_cb, 1)
        # Publisher for the switch states using an array of integers
        self.switch_publisher_ = self.create_publisher(
            Int32MultiArray, 'switch_states', 1)
        self.wheel_pub = self.create_publisher(DiffWheelCommands, 'wheel_commands', 10)

        # Set up a timer to call update_callback periodically (e.g., every 0.1 seconds)
        self.timer = self.create_timer(0.01, self.update_callback)

        # Stick
        self.ry = 0  # Vroom vroom (CH3)
        self.rx = 0  # steering (CH1)
        self.ly = 0  # Throttle (CH2)
        self.lx = 0  # we dont know (CH4)

        # Switch
        self.sw_a = 0
        self.sw_b = 0 
        self.sw_c = 0
        self.sw_d = 0
        
        #Add

        # Create subscriptions
        self.sub_ch1 = self.create_subscription(
            Int32, 'ch1', self.callback_1, 1)
        self.sub_ch2 = self.create_subscription(
            Int32, 'ch2', self.callback_2, 1)
        self.sub_ch3 = self.create_subscription(
            Int32, 'ch3', self.callback_3, 1)
        self.sub_ch4 = self.create_subscription(
            Int32, 'ch4', self.callback_4, 1)

        self.sub_ch5 = self.create_subscription(
            Int32, 'ch5', self.callback_5, 1)
        self.sub_ch6 = self.create_subscription(
            Int32, 'ch6', self.callback_6, 1)
        self.sub_ch7 = self.create_subscription(
            Int32, 'ch7', self.callback_7, 1)
        self.sub_ch8 = self.create_subscription(
            Int32, 'ch8', self.callback_8, 1)

    # Define separate callback functions for each channel
    def callback_1(self, msg): self.rx = msg.data
    def callback_2(self, msg): self.ry = msg.data
    def callback_3(self, msg): self.ly = msg.data
    def callback_4(self, msg): self.lx = msg.data

    def _auto_cmd_vel_cb(self, msg): self.auto_cmd_vel = msg

    def callback_5(self, msg): self.sw_a = msg.data
    def callback_6(self, msg): self.sw_b = msg.data
    def callback_7(self, msg): self.sw_c = msg.data
    def callback_8(self, msg): self.sw_d = msg.data

    def map_sw(self, value):
        if value == 1000:
            return 0
        elif value == 1500:
            return 1
        else:
            return 2

    def update_callback(self):

        # 0: Differentail Drive, 1: Fixed Heading
        self.drive_mode = self.map_sw(self.sw_c)
        #print(self.drive_mode)
        
        if self.drive_mode == 0:
            #self.get_logger().info("Differentail Drive")
            # Differentail Drive
            # L: Length (m), W: Width (m), max_speed: max speed (max speed is not used in the current implementation)
            vehicle = diff_drive(self.lx, self.ry)
            
            # calculates the rad_s for each wheel
            vehicle.rad_s_left = self.rad_s_calc(vehicle.v_left)
            vehicle.rad_s_right = self.rad_s_calc(vehicle.v_right)
            
            # self.get_logger().info(f"v_left: {vehicle.v_left} v_right {vehicle.v_right}")
            self.publish_wheels(vehicle)

        elif self.drive_mode == 1:
            self.publish_wheels_from_twist(self.auto_cmd_vel)

        # Publish the Switch state
        sw_msg = Int32MultiArray()
        sw_msg.data = [
            self.map_sw(self.sw_a),
            self.map_sw(self.sw_b),
            self.map_sw(self.sw_c),
            self.map_sw(self.sw_d)
        ]
        self.switch_publisher_.publish(sw_msg)
        
    def publish_wheels(self, vehicle):

        msg = DiffWheelCommands()
        msg.v_left = float(vehicle.rad_s_left)
        msg.v_right = float(vehicle.rad_s_right)
        
        self.wheel_pub.publish(msg)
    
    def publish_wheels_from_twist(self, twist):
        track_width = 0.5  # meters, distance between left and right wheels
        v     = twist.linear.x
        omega = twist.angular.z
        v_left  = v - omega * (track_width / 2.0)
        v_right = v + omega * (track_width / 2.0)

        msg = DiffWheelCommands()
        msg.v_left  = float(self.rad_s_calc(v_left))
        msg.v_right = float(self.rad_s_calc(v_right))
        self.wheel_pub.publish(msg)

    def publish_twist(self):
        input_range    = np.array([1000, 1480, 1520, 2000])
        linear_range   = np.array([-2.235, 0.0, 0.0,  2.235])
        angular_range  = np.array([ 1.5,   0.0, 0.0, -1.5  ])

        twist = Twist()
        twist.linear.x  = float(np.interp(self.ry, input_range, linear_range))
        twist.angular.z = float(np.interp(self.ly, input_range, angular_range))
        self.publisher_.publish(twist)

    def rad_s_calc(self, ms):
        max_ms = 2.235 # 5 mph
        wheel_radius_m = 0.2032 # 16" wheel
        
        ms = max(-max_ms,min(ms, max_ms))
        rad_s = ms / wheel_radius_m
        return rad_s
        


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
