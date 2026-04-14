from differentail_drive import DifferentialDrive

robot = DifferentialDrive(lx=1000, ly=1000, w=0.5, max_speed=2.0)

# Read the outputs you need
print(robot.v_left)   # → left wheel command
print(robot.v_right)  # → right wheel command
print(robot.omega)    # → angular velocity