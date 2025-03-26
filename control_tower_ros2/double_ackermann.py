import numpy as np
import matplotlib.pyplot as plt


class DoubleAckermannSteering:

    def __init__(self, lx, ly, l=0.711, w=0.558, max_speed=2.0):
        """
        Key Variable Units:

        - `self.lx`             : Raw lateral (steering) input from iBus (unitless, 1000 - 2000)
        - `self.ly`             : Raw longitudinal (throttle) input from iBus (unitless, 1000 - 2000)
        - `self.L`              : Vehicle length (meters, **m**)
        - `self.W`              : Vehicle width (meters, **m**)
        - `self.v`              : Vehicle's forward/reverse speed (meters per second, **m/s**)
        - `self.R`              : Turning radius (meters, **m**)
        - `self.str_angle`      : Steering angle of the centerline (radians)
        - `self.omega`          : Angular velocity (radians per second, **rad/s**)
        - `theta_f_left`, etc.  : Steering angles of each wheel (radians)
        - `v_f_left`, etc.      : Individual wheel velocities (meters per second, **m/s**)
        """
        
        self.lx = lx
        self.ly = ly
        self.l = l
        self.w = w
        self.max_speed = max_speed  # max forward/reverse speed
        self.v = 0  # Will be set based on ly and max_speed
        self.r = float("inf")
        self.str_angle = 0
        self.compute_steering()

    def omega(self):
        """Computes angular velocity (0 if moving straight)"""
        return 0 if self.r == float("inf") else self.v / self.r

    def compute_steering(self):
        """
        Maps:
        - lx ∈ [1000, 2000] → steering angle ∈ [-30°, +30°]
        - ly ∈ [1000, 2000] → velocity ∈ [-max_speed, +max_speed]
        """
        max_angle_deg = 30 # Maximum steering angle in degrees (don't exceed 50°)
        max_angle = np.radians(max_angle_deg)

        # Map lx to steering angle
        lx_clamped = np.clip(self.lx, 1000, 2000)
        normalized_lx = (lx_clamped - 1500) / 500.0  # [-1, 1]
        self.str_angle = normalized_lx * max_angle

        # Map ly to velocity
        ly_clamped = np.clip(self.ly, 1000, 2000)
        normalized_ly = (ly_clamped - 1500) / 500.0  # [-1, 1]
        self.v = normalized_ly * self.max_speed

        # Normalize and clamp lx input
        lx_clamped = np.clip(self.lx, 1000, 2000)

        # Map to range [-1, 1]
        normalized = (lx_clamped - 1500) / 500.0

        # Convert to steering angle in radians
        self.str_angle = normalized * max_angle

        # Compute turning radius or straight line
        if np.isclose(self.str_angle, 0, atol=np.radians(1)):
            self.r = float("inf")
        else:
            self.r = abs(self.l / np.tan(self.str_angle))

        turn_direction = np.sign(self.str_angle)

        # Straight-line motion
        if self.r == float("inf"):
            self.theta_f_right = 0
            self.theta_f_left = 0
            self.theta_r_right = 0
            self.theta_r_left = 0
            self.v_f_right = self.v
            self.v_f_left = self.v
            self.v_r_right = self.v
            self.v_r_left = self.v
        else:
            # Steering angles
            self.theta_f_right = turn_direction * \
                np.arctan2(self.l, (self.r - self.w / 2))
            self.theta_f_left = turn_direction * \
                np.arctan2(self.l, (self.r + self.w / 2))
            self.theta_r_right = -self.theta_f_right
            self.theta_r_left = -self.theta_f_left

            # Angular velocity
            self.omega = self.v / self.r

            # Wheel velocities
            self.v_f_right = self.omega * (self.r - self.w / 2)
            self.v_f_left = self.omega * (self.r + self.w / 2)
            self.v_r_right = self.v_f_right
            self.v_r_left = self.v_f_left

    def display_results(self):
        """Prints computed steering angles and wheel velocities"""
        print("Double Ackermann Steering Visualization:")
        print(f"Turning Angle R: {self.r}")
        print(f"Directional Angle in radians: {self.str_angle}")
        print(
            f"Front right Wheel Angle: {self.theta_f_right:.2f}° | Velocity: {self.v_f_right:.2f} m/s")
        print(
            f"Front left Wheel Angle: {self.theta_f_left:.2f}° | Velocity: {self.v_f_left:.2f} m/s")
        print(
            f"Rear right Wheel Angle: {self.theta_r_right:.2f}° | Velocity: {self.v_r_right:.2f} m/s")
        print(
            f"Rear left Wheel Angle: {self.theta_r_left:.2f}° | Velocity: {self.v_r_left:.2f} m/s")

    def visualize(self):
        """Visualizes the vehicle aligned along the Y-axis with steering angles in radians (displayed in degrees)."""

        # Define the vehicle body (facing +Y)
        vehicle_y = [-self.l / 2, self.l / 2,
                     self.l / 2, -self.l / 2, -self.l / 2]
        vehicle_x = [-self.w / 2, -self.w / 2,
                     self.w / 2, self.w / 2, -self.w / 2]

        # Define wheel positions [X, Y] in top-down view
        wheel_positions = np.array([
            [self.w / 2,  self.l / 2],   # Front Left
            [-self.w / 2,  self.l / 2],   # Front Right
            [self.w / 2, -self.l / 2],   # Rear Left
            [-self.w / 2, -self.l / 2],   # Rear Right
        ])

        # Convert wheel angles to radians (already in radians)
        angles_rad = [
            self.theta_f_right, self.theta_f_left,
            self.theta_r_right, self.theta_r_left
        ]

        # Compute direction vectors (X, Y) from angles
        wheel_vectors = np.array([
            [np.sin(ang), np.cos(ang)] for ang in angles_rad
        ])

        # Scale by velocity
        velocities = [
            self.v_f_right, self.v_f_left,
            self.v_r_right, self.v_r_left
        ]

        wheel_vectors *= np.array(velocities)[:, None]

        # Normalize for plotting
        max_v = max(abs(v) for v in velocities)
        if max_v > 0:
            wheel_vectors /= max_v  # normalize to length 1
        arrow_scale = 0.5
        wheel_vectors *= arrow_scale

        # Start plot
        plt.figure(figsize=(6, 8))
        plt.plot(vehicle_x, vehicle_y, 'k', linewidth=2, label="School Bus")

        # Draw axles
        plt.plot([-self.w / 2, self.w / 2],
                 [self.l / 2,  self.l / 2], 'b', linewidth=1)
        plt.plot([-self.w / 2, self.w / 2],
                 [-self.l / 2, -self.l / 2], 'b', linewidth=1)

        # Draw wheel vectors
        plt.quiver(
            wheel_positions[:, 0], wheel_positions[:, 1],
            wheel_vectors[:, 0], wheel_vectors[:, 1],
            color='r', angles='xy', scale_units='xy', scale=1, width=0.01
        )

        # Mark wheel positions
        plt.scatter(wheel_positions[:, 0],
                    wheel_positions[:, 1], color='k', zorder=3)

        # Annotate angles (in degrees for readability)
        plt.text(self.w / 2 + 0.2,  self.l / 2,
                 f"{np.degrees(self.theta_f_right):.1f}°", color='r', fontsize=12)
        plt.text(-self.w / 2 - 0.8,  self.l / 2,
                 f"{np.degrees(self.theta_f_left):.1f}°", color='r', fontsize=12)
        plt.text(self.w / 2 + 0.2, -self.l / 2,
                 f"{np.degrees(self.theta_r_right):.1f}°", color='r', fontsize=12)
        plt.text(-self.w / 2 - 0.8, -self.l / 2,
                 f"{np.degrees(self.theta_r_left):.1f}°", color='r', fontsize=12)

        # Labels and plot settings
        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")
        plt.title("Double Ackermann Steering Visualization (Facing Y+)")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        plt.show()


# Example Usage
if __name__ == "__main__":
    lx = 1500  # Left Stick X-axis input
    ly = 1500  # Left Stick Y-axis input
    vehicle = DoubleAckermannSteering(lx, ly, l=2.5, w=1.5, max_speed=2.0)
    vehicle.display_results()
    vehicle.visualize()
