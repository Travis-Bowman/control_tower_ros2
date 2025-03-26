import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import numpy as np
from control_tower_ros2.double_ackermann import DoubleAckermannSteering as da

from paho.mqtt import client as MQTT


class control_tower_node(Node):
    
    mqtt_host = "192.168.0.3"
    mqtt_port = 1883
    mqtt_name = "controlTower"
    mqtt_client : MQTT.Client
    
    def __init__(self):
        super().__init__('control_tower_node')
        # Create a publisher for the Twist message on the 'cmd_vel' topic.
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
        # Publisher for the switch states using an array of integers
        self.switch_publisher_ = self.create_publisher(
            Int32MultiArray, 'switch_states', 1)
        # Set up a timer to call update_callback periodically (e.g., every 0.1 seconds)
        self.timer = self.create_timer(0.01, self.update_callback)

        # Stick
        self.ry = 0  # Vroom vroom (CH3)
        self.rx = 0  # steering (CH1)
        self.ly = 0  # Throttle (CH2)
        self.lx = 0  # we dont know (CH4)

        # Switch
        self.sw_a = 0
        self.sw_b = 0  # broken
        self.sw_c = 0
        self.sw_d = 0

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
        
        # set up MQTT client
        self.mqtt_client = MQTT.Client(client_id=self.mqtt_name)
        self.mqtt_client.connect(self.mqtt_host, self.mqtt_port)
        self.mqtt_client.loop_start()

    # Define separate callback functions for each channel
    def callback_1(self, msg): self.rx = msg.data
    def callback_2(self, msg): self.ly = msg.data
    def callback_3(self, msg): self.ry = msg.data
    def callback_4(self, msg): self.lx = msg.data

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

        # 0: double Ackermann, 1: Fixed Heading, 2: edu-bot test mode
        self.drive_mode = self.sw_c

        if self.drive_mode == 0:
            # Double Ackermann
            # L: Length (m), W: Width (m), max_speed: max speed (max speed is not used in the current implementation)
            vehicle = da(self.lx, self.ly)
            self.publish_wheels(vehicle)
            # TODO: Publish wheel angles/velocities from vehicle object
            # vehicle.theta_f_left
            # vehicle.theta_f_right
            # vehicle.theta_r_left
            # vehicle.theta_r_right
            # vehicle.v_f_left
            # vehicle.v_f_right
            # vehicle.v_r_left
            # vehicle.v_r_right

            # Debugging
            # self.get_logger().info(f"Published Wheel Angles: {vehicle.theta_f_left}, {vehicle.theta_f_right}, {vehicle.theta_r_left}, {vehicle.theta_r_right}")
            # self.get_logger().info(f"Published Wheel Velocities: {vehicle.v_f_left}, {vehicle.v_f_right}, {vehicle.v_r_left}, {vehicle.v_r_right}")

        elif self.drive_mode == 1:
            # Fixed Heading
            pass
        elif self.drive_mode == 2:
            # edu-bot test mode
            input_range = np.array([1000, 1450, 1550, 2000])
            linear_output_range = np.array([-10, 0, 0, 10])
            angular_output_range = np.array([-3.14, 0, 0, 3.14])

            # Map the IBUS data to a Twist message.
            twist_msg = Twist()
            twist_msg.linear.x = np.interp(
                self.ly, input_range, linear_output_range)
            twist_msg.angular.z = np.interp(
                self.lx, input_range, angular_output_range)

            # Publish the Twist message
            self.publisher_.publish(twist_msg)

            # Debuging
            # self.get_logger().info(f"Published Twist: {twist_msg}")

        # Publish the Switch state
        sw_msg = Int32MultiArray()
        sw_msg.data = [
            self.map_sw(self.sw_a),
            self.map_sw(self.sw_b),
            self.map_sw(self.sw_c),
            self.map_sw(self.sw_d)
        ]
        self.switch_publisher_.publish(sw_msg)
        # Debugging
        # self.get_logger().info(f"Published Switch States: {sw_msg.data}")
        
    def publish_wheels(self, vehicle : da):
        self.mqtt_client.publish("/frontleft/power", vehicle.v_f_left)
        self.mqtt_client.publish("/frontleft/steer", vehicle.theta_f_left)
        
        self.mqtt_client.publish("/frontright/power", vehicle.v_f_right)
        self.mqtt_client.publish("/frontright/steer", vehicle.theta_f_right)
        
        self.mqtt_client.publish("/backleft/power", vehicle.v_r_left)
        self.mqtt_client.publish("/backleft/steer", vehicle.theta_r_left)
        
        self.mqtt_client.publish("/backright/power", vehicle.v_r_right)
        self.mqtt_client.publish("/backright/steer", vehicle.theta_r_right)


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
