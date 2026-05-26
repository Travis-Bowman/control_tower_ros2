#!/usr/bin/env python3
"""
state_feedback_controller.py
 
Pure-Python controller + EKF — no ROS dependency.
Instantiate StateFeedbackController in any node and call step() each tick.
 
Example
-------
    from controller import StateFeedbackController
    import numpy as np
 
    ctrl = StateFeedbackController()
 
    # Inside your timer callback:
    voltages, pose = ctrl.step(
        y          = np.array([w_left_rad_s, w_right_rad_s]),
        r          = np.array([ref_left_rad_s, ref_right_rad_s]),
        imu_yaw    = float_or_None,   # pass None when no new sample
        imu_gyro_z = float_or_None,
    )
    # voltages : np.ndarray shape (2,)  [V_left, V_right], clipped to ±V_MAX_V
    # pose     : np.ndarray shape (4,)  [X_m, Y_m, theta_rad, gyro_bias_rad_s]
"""
 
import math
import numpy as np
 
 
# ---------------------------------------------------------------------------
# SYSTEM PHYSICAL PARAMETERS
# ---------------------------------------------------------------------------
 
WHEEL_RADIUS_M = 0.2032   # Wheel radius [m]  (8 in)
WHEEL_BASE_M   = 0.15     # Axle length  [m]
 
N     = 20.0              # Gear ratio
R_MOT = 0.2               # Motor resistance [Ω]
K_E   = 0.0468            # Back-EMF constant [V·s/rad]
K_T   = K_E               # Torque constant   [N·m/A]
J     = 1.8581            # Effective inertia  [kg·m²]
 
DT_S    = 0.01            # Control period [s]  →  100 Hz
V_MAX_V = 24.0            # Hard voltage saturation [V]
 
 
# ---------------------------------------------------------------------------
# CONTROLLER GAINS  (from MATLAB, with manual right-wheel scaling)
# ---------------------------------------------------------------------------
 
_K_FB_RAW = np.array([
    [19.8477, -2.9176],
    [ 3.9802, 17.8784],
], dtype=float)
 
_KI_RAW = np.array([
    [-216.7824,  125.1466],
    [-152.2550,  -67.6913],
], dtype=float)
 
K_FB = _K_FB_RAW.copy()
K_FB[1, 1] *= 2.0          # manual right-wheel tuning
 
KI = _KI_RAW.copy()
KI[1, 1] *= 3.0
 
 
# ---------------------------------------------------------------------------
# EKF NOISE TUNING
# ---------------------------------------------------------------------------
 
EKF_Q = np.diag([1e-5, 1e-5, 5e-4, 1e-6])   # process noise  — TUNE
EKF_P0 = np.diag([1e-3, 1e-3, 1e-3, 1e-4])  # initial state covariance
 
EKF_R_YAW  = 1.22e-3    # yaw measurement noise variance  [rad²]
EKF_R_GYRO = 7.62e-7    # gyro-Z measurement noise variance [rad²/s²]
 
 
# ---------------------------------------------------------------------------
# Controller class
# ---------------------------------------------------------------------------
 
class StateFeedbackController:
    """
    Stateful state-feedback + integral controller with a 4-state EKF
    (X, Y, theta, gyro_bias).
 
    All ROS plumbing is the caller's responsibility.  This class only
    holds numeric state and implements one deterministic step() call.
    """
 
    def __init__(self):
        # Integrator state  [left, right]  [rad]
        self._xi = np.zeros(2)
 
        # EKF state  [X_m, Y_m, theta_rad, gyro_bias_rad_s]
        self._s = np.zeros(4)
        self._P = EKF_P0.copy()
 
    def reset(self):
        """Zero integrators and EKF state (e.g. on E-stop / re-enable)."""
        self._xi[:] = 0.0
        self._s[:] = 0.0
        self._P[:] = EKF_P0
 
    # ------------------------------------------------------------------
    # Main callable
    # ------------------------------------------------------------------
 
    def step(
        self,
        y:          np.ndarray,         # measured wheel speeds [rad/s]  shape (2,)
        r:          np.ndarray,         # reference wheel speeds [rad/s] shape (2,)
        imu_yaw:    float | None,       # yaw from IMU [rad],   None = no new sample
        imu_gyro_z: float | None,       # gyro-Z [rad/s],       None = no new sample
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Run one EKF + control step.
 
        Parameters
        ----------
        y          : measured wheel speeds [rad/s],  [left, right]
        r          : reference wheel speeds [rad/s], [left, right]
        imu_yaw    : absolute yaw from IMU [rad],  or None
        imu_gyro_z : angular rate from gyro [rad/s], or None
 
        Returns
        -------
        voltages : np.ndarray (2,)  — [V_left, V_right], saturated to ±V_MAX_V
        pose     : np.ndarray (4,)  — [X_m, Y_m, theta_rad, gyro_bias_rad_s]
        """
        y = np.asarray(y, dtype=float)
        r = np.asarray(r, dtype=float)
 
        # ---- EKF predict --------------------------------------------------
        v_l = y[0] * WHEEL_RADIUS_M
        v_r = y[1] * WHEEL_RADIUS_M
        v         = (v_r + v_l) / 2.0
        omega_enc = (v_r - v_l) / WHEEL_BASE_M
 
        theta  = self._s[2]
        s_pred = self._s.copy()
        s_pred[0] += v * math.cos(theta) * DT_S
        s_pred[1] += v * math.sin(theta) * DT_S
        s_pred[2]  = _wrap_to_pi(s_pred[2] + omega_enc * DT_S)
        # s_pred[3] constant — gyro bias modelled as a slowly varying constant
 
        F      = np.eye(4)
        F[0, 2] = -v * math.sin(theta) * DT_S
        F[1, 2] =  v * math.cos(theta) * DT_S
        P_pred  = F @ self._P @ F.T + EKF_Q
 
        # ---- EKF update: yaw measurement ----------------------------------
        if imu_yaw is not None:
            H     = np.array([[0.0, 0.0, 1.0, 0.0]])
            innov = _wrap_to_pi(imu_yaw - s_pred[2])
            S     = float(H @ P_pred @ H.T) + EKF_R_YAW
            K     = (P_pred @ H.T) / S
            s_pred  = s_pred + K.flatten() * innov
            s_pred[2] = _wrap_to_pi(s_pred[2])
            P_pred  = (np.eye(4) - K @ H) @ P_pred
 
        # ---- EKF update: gyro-Z measurement -------------------------------
        if imu_gyro_z is not None:
            Hg      = np.array([[0.0, 0.0, 0.0, 1.0]])
            innov_g = imu_gyro_z - (omega_enc + s_pred[3])
            Sg      = float(Hg @ P_pred @ Hg.T) + EKF_R_GYRO
            Kg      = (P_pred @ Hg.T) / Sg
            s_pred  = s_pred + Kg.flatten() * innov_g
            P_pred  = (np.eye(4) - Kg @ Hg) @ P_pred
 
        self._s = s_pred
        self._P = P_pred
 
        # ---- Control law --------------------------------------------------
        e = r - y
        u = -(K_FB @ y) - (KI @ self._xi)
        u = np.clip(u, -V_MAX_V, V_MAX_V)
 
        # Integrator update with conditional anti-windup
        if abs(u[0]) < V_MAX_V:
            self._xi[0] += e[0] * DT_S
        if abs(u[1]) < V_MAX_V:
            self._xi[1] += e[1] * DT_S
 
        return u, self._s.copy()
 
 
# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------
 
def _wrap_to_pi(angle: float) -> float:
    """Wrap an angle to [-π, π]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
 