#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int32, Float64, Float64MultiArray
from geometry_msgs.msg import Twist
import numpy as np
from control_tower_ros2.differentail_drive import DifferentialDrive as diff_drive
from control_tower_ros2.msg import DiffWheelCommands
from control_tower_ros2.controller import StateFeedbackController

WHEEL_RADIUS_M = 0.2032  # must match controller.py

try:
    from gpiozero import LED
    _GPIO_AVAILABLE = True
except Exception:
    _GPIO_AVAILABLE = False

SAFETY_LIGHT_PIN = 26

class control_tower_node(Node):
    

    def __init__(self):
        super().__init__('control_tower_node')
        # Create a publisher for the Twist message on the 'cmd_vel' topic.
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
        # Publisher for the switch states using an array of integers
        self.switch_publisher_ = self.create_publisher(
            Int32MultiArray, 'switch_states', 1)
        self.wheel_pub = self.create_publisher(DiffWheelCommands, 'wheel_commands', 10)

        self.declare_parameter('left_trim', 1.0)
        self.declare_parameter('right_trim', 1.0)
        self.declare_parameter('wheel_base', 0.558)

        try:
            self.safety_light = LED(SAFETY_LIGHT_PIN) if _GPIO_AVAILABLE else None
        except Exception:
            self.safety_light = None
        self.drive_mode = 0
        self._light_tick = 0

        # State-feedback controller + EKF
        self._ctrl = StateFeedbackController()
        self._wheel_measured = [0.0, 0.0]
        self._imu_yaw = None
        self._imu_gyro_z = None
        self.create_subscription(DiffWheelCommands, '/wheel_feedback', self._enc_cb, 10)
        self.create_subscription(Float64, '/imu/yaw',    self._imu_yaw_cb,  10)
        self.create_subscription(Float64, '/imu/gyro_z', self._imu_gyro_cb, 10)
        self._pose_pub = self.create_publisher(Float64MultiArray, '/pose_estimate', 10)

        # Set up a timer to call update_callback periodically (e.g., every 0.1 seconds)
        self.timer = self.create_timer(0.01, self.update_callback)

        # Stick — default to 1500 (RC center) so no-receiver = zero command
        self.ry = 1500  # Vroom vroom (CH3)
        self.rx = 1500  # steering (CH1)
        self.ly = 1500  # Throttle (CH2)
        self.lx = 1500  # we dont know (CH4)

        # Switch
        self.sw_a = 0
        self.sw_b = 0
        self.sw_c = 1000  # default to teleop before RC receiver connects
        self.sw_d = 0

        # Autonomous (lane-follow) Twist command — updated by /cmd_vel subscriber
        self._auto_twist = None
        self.create_subscription(Twist, '/cmd_vel', self._auto_cmd_vel_cb, 1)

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

    def callback_5(self, msg): self.sw_a = msg.data
    def callback_6(self, msg): self.sw_b = msg.data
    def callback_7(self, msg): self.sw_c = msg.data
    def callback_8(self, msg): self.sw_d = msg.data
    def _auto_cmd_vel_cb(self, msg): self._auto_twist = msg

    def _enc_cb(self, msg):
        self._wheel_measured = [float(msg.v_left), float(msg.v_right)]  # m/s

    def _imu_yaw_cb(self, msg):    self._imu_yaw    = float(msg.data)
    def _imu_gyro_cb(self, msg):   self._imu_gyro_z = float(msg.data)

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

        if self.safety_light:
            if self.drive_mode == 0:
                self._light_tick = 0
                self.safety_light.on()
            else:
                self._light_tick = (self._light_tick + 1) % 100
                if self._light_tick == 0:
                    self.safety_light.on()
                elif self._light_tick == 50:
                    self.safety_light.off()

        # self.get_logger().info(f'drive_mode={self.drive_mode} sw_c={self.sw_c} tick={self._light_tick}', throttle_duration_sec=1.0)

        if self.drive_mode == 0:
            #self.get_logger().info("Differentail Drive")
            # Differentail Drive
            # L: Length (m), W: Width (m), max_speed: max speed (max speed is not used in the current implementation)
            vehicle = diff_drive(self.lx, self.ry)
            v_left, v_right = self._run_controller(vehicle)
            self.get_logger().info(f'v_left={v_left} v_right={v_right}', throttle_duration_sec=1.0)
            self.publish_wheels(v_left, v_right)

        elif self.drive_mode == 1:
            # Fixed Heading
            pass

        # Publish the Switch state
        sw_msg = Int32MultiArray()
        sw_msg.data = [
            self.map_sw(self.sw_a),
            self.map_sw(self.sw_b),
            self.map_sw(self.sw_c),
            self.map_sw(self.sw_d)
        ]
        self.switch_publisher_.publish(sw_msg)

    def publish_wheels(self, v_left: float, v_right: float):
        msg = DiffWheelCommands()
        msg.v_left  = v_left
        msg.v_right = v_right
        self.wheel_pub.publish(msg)

    def _run_controller(self, vehicle) -> tuple:
        left_trim  = self.get_parameter('left_trim').get_parameter_value().double_value
        right_trim = self.get_parameter('right_trim').get_parameter_value().double_value

        # Reference: joystick m/s (with trim) → rad/s
        r = np.array([
            vehicle.v_left  * left_trim  / WHEEL_RADIUS_M,
            vehicle.v_right * right_trim / WHEEL_RADIUS_M,
        ])
        # Measured: encoder feedback m/s → rad/s
        y = np.array(self._wheel_measured) / WHEEL_RADIUS_M

        voltages, pose = self._ctrl.step(
            y=y, r=r,
            imu_yaw=self._imu_yaw,
            imu_gyro_z=self._imu_gyro_z,
        )
        self._imu_yaw    = None
        self._imu_gyro_z = None

        # Voltage → m/s:  24 V = 2.0 m/s = 100% throttle  →  ÷ 12
        v_left  = float(np.clip(voltages[0], -24.0, 24.0)) / 12.0
        v_right = float(np.clip(voltages[1], -24.0, 24.0)) / 12.0

        p_msg = Float64MultiArray()
        p_msg.data = pose.tolist()
        self._pose_pub.publish(p_msg)

        return v_left, v_right


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

