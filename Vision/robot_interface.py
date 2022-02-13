import serial
import serial.tools.list_ports
import time

# List all serial ports
ports = [port.device for port in serial.tools.list_ports.comports()]
print(f"Serial ports: ${ports}")

# Connect to first available serial port with 115200 baud
ser = serial.Serial(port=ports[-1], baudrate=115200, timeout=0.001)

robots_still_working = {}


def drive(robot_num, x, y, th):
    """Drive to pose x, y, th in world coordinates.

    Args:
        robot_num (int): Robot ID number.
        x (float): X coordinate in mm.
        y (float): Y coordinate in mm.
        th (float): Angle in radians.
    """
    ser.write(f'R{robot_num};D;X{x/1000};Y{y/1000};T{th}\n'.encode('utf-8'))
    robots_still_working[robot_num] = True


def reverse(robot_num, x, y, th):
    """Drive to pose x, y, th BACKWARDS in world coordinates.

    Args:
        robot_num (int): Robot ID number.
        x (float): X coordinate in mm.
        y (float): Y coordinate in mm.
        th (float): Angle in radians.
    """
    ser.write(f'R{robot_num};B;X{x/1000};Y{y/1000};T{th}\n'.encode('utf-8'))
    robots_still_working[robot_num] = True


def set_pose(robot_num, x, y, th):
    """Overwrite pose of the given robot to x, y, th in world coordinates.

    Args:
        robot_num (int): Robot ID number.
        x (float): X coordinate in mm.
        y (float): Y coordinate in mm.
        th (float): Angle in radians.
    """
    ser.write(f'R{robot_num};W;X{x/1000};Y{y/1000};T{th}\n'.encode('utf-8'))


def refine_pose(robot_num, x, y, th):
    """Refine pose of the given robot to x, y, th in world coordinates.
    In other words, the robot will do a weighted average of its current
    (estimated using odometry) pose and the given pose.

    Args:
        robot_num (int): Robot ID number.
        x (float): X coordinate in mm.
        y (float): Y coordinate in mm.
        th (float): Angle in radians.
    """
    ser.write(f'R{robot_num};U;X{x/1000};Y{y/1000};T{th}\n'.encode('utf-8'))


def pick(robot_num):
    """Pick up the object using the gripper.

    Args:
        robot_num (int): Robot ID number.
    """
    ser.write(f'R{robot_num};P\n'.encode('utf-8'))
    robots_still_working[robot_num] = True


def place(robot_num):
    """Place the object using the gripper.

    Args:
        robot_num (int): Robot ID number.
    """
    ser.write(f'R{robot_num};L\n'.encode('utf-8'))
    robots_still_working[robot_num] = True


def isRobotRunning(robot_num):
    """Returns True if robot is running, False otherwise.

    Args:
        robot_num (int): Robot ID number.
    """

    # Read incoming data from serial port
    data = ser.read_until().decode('utf-8')
    if len(data) > 0:
        print(f"RECEIVED DATA FROM ROBOT: \"{data}\"")

    if ";DONE" in data:
        # Robot is done moving
        robot_num = int(data[1])
        robots_still_working[robot_num] = False

    if robot_num in robots_still_working:
        return robots_still_working[robot_num]
    else:
        return False


def wait(robot_num):
    """Wait for the given robot to finish moving.

    Args:
        robot_num (int): Robot ID number.
    """
    while(isRobotRunning(robot_num)):
        time.sleep(0.1)


if __name__ == "__main__":
    for k in range(10):
        print(isRobotRunning(0))
        if k == 2:
            pick(0)
        time.sleep(2)
