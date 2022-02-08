import serial.tools.list_ports

# List all serial ports
ports = [port.device for port in serial.tools.list_ports.comports()]
print(f"Serial ports: ${ports}")

# Connect to first available serial port with 115200 baud
ser = serial.Serial(port=ports[-1], baudrate=115200)


def drive(robot_num, x, y, th):
    """Drive to pose x, y, th in world coordinates.

    Args:
        robot_num (int): Robot ID number.
        x (float): X coordinate in meters.
        y (float): Y coordinate in meters.
        th (float): Angle in radians.
    """
    ser.write(f'R{robot_num};D;X{x};Y{y};T{th}\n'.encode('utf-8'))

def set_pose(robot_num, x, y, th):
    """Overwrite pose of the given robot to x, y, th in world coordinates.

    Args:
        robot_num (int): Robot ID number.
        x (float): X coordinate in meters.
        y (float): Y coordinate in meters.
        th (float): Angle in radians.
    """
    ser.write(f'R{robot_num};W;X{x};Y{y};T{th}\n'.encode('utf-8'))

def refine_pose(robot_num, x, y, th):
    """Refine pose of the given robot to x, y, th in world coordinates.
    In other words, the robot will do a weighted average of its current
    (estimated using odometry) pose and the given pose.

    Args:
        robot_num (int): Robot ID number.
        x (float): X coordinate in meters.
        y (float): Y coordinate in meters.
        th (float): Angle in radians.
    """
    ser.write(f'R{robot_num};U;X{x};Y{y};T{th}\n'.encode('utf-8'))

def pick(robot_num):
    """Pick up the object using the gripper.

    Args:
        robot_num (int): Robot ID number.
    """
    ser.write(f'R{robot_num};P\n'.encode('utf-8'))

def place(robot_num):
    """Place the object using the gripper.

    Args:
        robot_num (int): Robot ID number.
    """
    ser.write(f'R{robot_num};L\n'.encode('utf-8'))
    