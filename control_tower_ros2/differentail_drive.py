import numpy as np


class DifferentialDrive:

    def __init__(self, lx, ry, w=0.558, max_speed=11.0):
        """
        Key Variable Units:

        - `self.lx`             : Raw lateral (turn) input from iBus (unitless, 1000 - 2000)
        - `self.ry`             : Raw longitudinal (throttle) input from iBus (unitless, 1000 - 2000)
        - `self.W`              : Distance between left and right wheels (meters, **m**)
        - `self.v`              : Vehicle's forward/reverse linear speed (meters per second, **m/s**)
        - `self.omega`          : Angular velocity of the vehicle (radians per second, **rad/s**)
        - `self.R`              : Turning radius about the ICC (meters, **m**; inf when driving straight)
        - `self.v_left`         : Left wheel velocity (meters per second, **m/s**)
        - `self.v_right`        : Right wheel velocity (meters per second, **m/s**)
        """
        # Differential drive

        self.lx = lx
        self.ry = ry
        self.w = w
        self.max_speed = max_speed      # max forward/reverse linear speed (m/s)
        self.v = 0                      # linear speed, set in compute_steering
        self.omega = 0                  # angular velocity, set in compute_steering
        self.r = float("inf")           # turning radius
        self.v_left = 0
        self.v_right = 0
        self.compute_steering()


   
    def compute_steering(self):
        # Map ry → base speed  (you pass this in as the second argument)
        ry_clamped = np.clip(self.ry, 1000, 2000)          # ry holds the ry value
        normalized_ry = (ry_clamped - 1000) / 1000.0       # [0, 1]  (or [-1,1] if reversed)
        v_base = (normalized_ry - 0.5) * 2 * self.max_speed  # centre-stick = 0

        # Map lx → differential (positive lx = turn right = slow right wheel)
        lx_clamped = np.clip(self.lx, 1000, 2000)
        normalized_lx = (lx_clamped - 1000) / 1000.0       # [0, 1]
        v_diff = (normalized_lx - 0.5) * 2 * self.max_speed # centre-stick = 0

        # Turn right: left wheel faster, right wheel slower → subtract diff from right
        self.v_left  = np.clip(v_base + v_diff, -self.max_speed, self.max_speed)
        self.v_right = np.clip(v_base - v_diff, -self.max_speed, self.max_speed)
 
        # Derived quantities
        self.v = (self.v_right + self.v_left) / 2.0

        diff = self.v_right - self.v_left
        if np.isclose(diff, 0, atol=1e-6):
            self.omega = 0.0
            self.r = float("inf")
        else:
            self.omega = diff / self.w
            self.r = (self.w / 2) * (self.v_left + self.v_right) / diff

    def display_results(self):
        """Prints computed wheel velocities and turning geometry."""
        print("Differential Drive:")
        print(f"  Linear velocity  v     : {self.v:.4f} m/s")
        print(f"  Angular velocity omega : {self.omega:.4f} rad/s  "
              f"({np.degrees(self.omega):.2f} deg/s)")
        print(f"  Turning radius   R     : "
              f"{'inf (straight)' if self.r == float('inf') else f'{self.r:.4f} m'}")
        print(f"  Left  wheel velocity   : {self.v_left:.4f} m/s")
        print(f"  Right wheel velocity   : {self.v_right:.4f} m/s")


# Example Usage
if __name__ == "__main__":
    lx = 1600   # Steering stick — right of centre → turn right
    ry = 1500   # Throttle stick — above centre     → forward
    robot = DifferentialDrive(lx, ry, w=0.5, max_speed=2.0)
    robot.display_results()
