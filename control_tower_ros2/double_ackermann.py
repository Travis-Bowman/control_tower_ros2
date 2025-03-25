import numpy as np
import matplotlib.pyplot as plt

class DoubleAckermannSteering:

    def __init__(self, lx, ly, L=2.5, W=1.5, v=2, R=10):
        """
        Initializes the Ackermann steering model.

        Parameters:
        - lx: left stick X axis
        - ly: left stick y axis
        - L: Wheelbase (m)
        - W: Track width (m)
        - v: Forward velocity (m/s)
        - R: Desired turning radius (m) (Set to float('inf') for straight-line motion)
        """
        self.lx = lx
        self.ly = ly
        self.L = L
        self.W = W
        self.v = v
        self.R = R
        self.str_angle = 0


        # Compute steering angles and wheel velocities
        self.compute_steering()

    @property
    def omega(self):
        """Computes angular velocity (0 if moving straight)"""
        return 0 if self.R == float("inf") else self.v / self.R


    def compute_steering(self):
        """Computes the Ackermann steering angles and wheel velocities with correct left/right turning behavior."""

        # Compute Steering Angle Based on (lx, ly) Direction
        self.str_angle = np.arctan2(self.ly, self.lx)  # Angle in radians

        # Compute Turning Radius (Ensure R is Positive)
        self.R = abs(self.L / np.tan(self.str_angle)) if not np.isclose(self.str_angle, 0) else float("inf")

        # Adjust Steering Direction for Left vs Right Turns
        turn_direction = np.sign(self.str_angle)  # +1 for Right, -1 for Left

    # Handle Straight-Line Motion
        if self.R == float("inf"):
            self.theta_f_inner = 0
            self.theta_f_outer = 0
            self.theta_r_inner = 0
            self.theta_r_outer = 0
        else:
            # Compute Ackermann Steering Angles (in Degrees)
            self.theta_f_inner = turn_direction * np.degrees(np.arctan2(self.L, (self.R - self.W / 2)))
            self.theta_f_outer = turn_direction * np.degrees(np.arctan2(self.L, (self.R + self.W / 2)))
            self.theta_r_inner = -self.theta_f_inner  # Rear inner opposite to front
            self.theta_r_outer = -self.theta_f_outer  # Rear outer opposite to front

        # Compute individual wheel velocities
            self.v_f_inner = self.omega * (self.R - self.W / 2)
            self.v_f_outer = self.omega * (self.R + self.W / 2)
            self.v_r_inner = self.omega * (self.R - self.W / 2)
            self.v_r_outer = self.omega * (self.R + self.W / 2)

    def display_results(self):
        """Prints computed steering angles and wheel velocities"""
        print("Double Ackermann Steering Visualization:")
        print(f"Turning Angle R: {self.R}")
        print(f"Directional Angle in radians: {self.str_angle}")
        print(f"Front Inner Wheel Angle: {self.theta_f_inner:.2f}° | Velocity: {self.v_f_inner:.2f} m/s")
        print(f"Front Outer Wheel Angle: {self.theta_f_outer:.2f}° | Velocity: {self.v_f_outer:.2f} m/s")
        print(f"Rear Inner Wheel Angle: {self.theta_r_inner:.2f}° | Velocity: {self.v_r_inner:.2f} m/s")
        print(f"Rear Outer Wheel Angle: {self.theta_r_outer:.2f}° | Velocity: {self.v_r_outer:.2f} m/s")

    def visualize(self):
      """Visualizes the vehicle body, wheels, and steering directions"""
      # Vehicle Body Outline
      vehicle_x = [-self.L / 2, self.L / 2, self.L / 2, -self.L / 2, -self.L / 2]
      vehicle_y = [-self.W / 2, -self.W / 2, self.W / 2, self.W / 2, -self.W / 2]

      # Wheel Positions
      wheel_positions = np.array([
          [self.L / 2, self.W / 2],  # Front Left
          [self.L / 2, -self.W / 2],  # Front Right
          [-self.L / 2, self.W / 2],  # Rear Left
          [-self.L / 2, -self.W / 2]  # Rear Right
      ])

      # Convert Angles to Radians for Plotting
      theta_f_inner_rad = np.radians(self.theta_f_inner)
      theta_f_outer_rad = np.radians(self.theta_f_outer)
      theta_r_inner_rad = np.radians(self.theta_r_inner)
      theta_r_outer_rad = np.radians(self.theta_r_outer)

      # Compute Wheel Direction Vectors (Scaled by Velocity)
      arrow_scale = 0.5
      wheel_vectors = np.array([
          [np.cos(theta_f_inner_rad), np.sin(theta_f_inner_rad), self.v_f_inner],  # Front Left
          [np.cos(theta_f_outer_rad), np.sin(theta_f_outer_rad), self.v_f_outer],  # Front Right
          [np.cos(theta_r_inner_rad), np.sin(theta_r_inner_rad), self.v_r_inner],  # Rear Left
          [np.cos(theta_r_outer_rad), np.sin(theta_r_outer_rad), self.v_r_outer]   # Rear Right
      ])

      # Normalize Vector Lengths
      max_v = np.max(np.abs(wheel_vectors[:, 2]))
      if max_v > 0:
          wheel_vectors[:, :2] = (wheel_vectors[:, :2] * wheel_vectors[:, 2][:, None]) / max_v

      # Plot Vehicle Body
      plt.figure(figsize=(7, 5))
      plt.plot(vehicle_x, vehicle_y, 'k', linewidth=2, label="Vehicle Body")

      # Plot Wheels as Arrows (Red for Steering Direction)
      plt.quiver(
          wheel_positions[:, 0], wheel_positions[:, 1],
          arrow_scale * wheel_vectors[:, 0], arrow_scale * wheel_vectors[:, 1],
          color='r', angles='xy', scale_units='xy', scale=1, width=0.01
      )

        # Plot Axles
      plt.plot([self.L / 2, self.L / 2], [-self.W / 2, self.W / 2], 'b', linewidth=1)
      plt.plot([-self.L / 2, -self.L / 2], [-self.W / 2, self.W / 2], 'b', linewidth=1)

        # Labels
      plt.xlabel("X Position (m)")
      plt.ylabel("Y Position (m)")
      plt.title("Double Ackermann Steering - Wheel Angle & Velocity Visualization")
      plt.legend(["Vehicle Body", "Wheel Steering Direction"], loc="best")

        # Mark Wheel Positions
      plt.scatter(wheel_positions[:, 0], wheel_positions[:, 1], color='k', zorder=3)

        # Text Labels for Angles
      plt.text(self.L / 2 + 0.2, self.W / 2, f"{self.theta_f_inner:.1f}°", color='r', fontsize=12)
      plt.text(self.L / 2 + 0.2, -self.W / 2, f"{self.theta_f_outer:.1f}°", color='r', fontsize=12)
      plt.text(-self.L / 2 - 0.5, self.W / 2, f"{self.theta_r_inner:.1f}°", color='r', fontsize=12)
      plt.text(-self.L / 2 - 0.5, -self.W / 2, f"{self.theta_r_outer:.1f}°", color='r', fontsize=12)

      plt.axis("equal")
      plt.grid(True)
      plt.show()

# Example Usage
if __name__ == "__main__":
    lx = 0
    ly = 0
    vehicle = DoubleAckermannSteering(lx, ly, L=2.5, W=1.5, v=2, R=10)
    vehicle.display_results()
    vehicle.visualize()
