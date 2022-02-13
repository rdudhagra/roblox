import serial
import serial.tools.list_ports
import time

# List all serial ports
ports = [port.device for port in serial.tools.list_ports.comports()]
print(f"Serial ports: ${ports}")

# Connect to first available serial port with 115200 baud
ser = serial.Serial(port=ports[-1], baudrate=115200, timeout=0.02)
last_cmd = ""

robots_still_working = {}

timeout_time = {}


def drive(robot_num, x, y, th):
    """Drive to pose x, y, th in world coordinates.

    Args:
        robot_num (int): Robot ID number.
        x (float): X coordinate in mm.
        y (float): Y coordinate in mm.
        th (float): Angle in radians.
    """
    global timeout_time

    last_cmd = f'R{robot_num};D;X{x/1000};Y{y/1000};T{th}\n'
    ser.write(last_cmd.encode('utf-8'))
    robots_still_working[robot_num] = True

    timeout_time[robot_num] = time.time() + 15


def reverse(robot_num, x, y, th):
    """Drive to pose x, y, th BACKWARDS in world coordinates.

    Args:
        robot_num (int): Robot ID number.
        x (float): X coordinate in mm.
        y (float): Y coordinate in mm.
        th (float): Angle in radians.
    """
    global timeout_time

    last_cmd = f'R{robot_num};B;X{x/1000};Y{y/1000};T{th}\n'
    ser.write(last_cmd.encode('utf-8'))
    robots_still_working[robot_num] = True

    timeout_time[robot_num] = time.time() + 15


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
    global timeout_time

    last_cmd = f'R{robot_num};P\n'
    ser.write(last_cmd.encode('utf-8'))
    robots_still_working[robot_num] = True

    timeout_time[robot_num] = time.time() + 5


def place(robot_num):
    """Place the object using the gripper.

    Args:
        robot_num (int): Robot ID number.
    """
    global timeout_time

    last_cmd = f'R{robot_num};L\n'
    ser.write(last_cmd.encode('utf-8'))
    robots_still_working[robot_num] = True

    timeout_time[robot_num] = time.time() + 5


serialData = ""


def isRobotRunning(robot_num):
    """Returns True if robot is running, False otherwise.

    Args:
        robot_num (int): Robot ID number.
    """
    global serialData

    # Read incoming data from serial port
    data = ser.read_until().decode('utf-8')
    if len(data) > 0:
        print(f"RECEIVED DATA FROM ROBOT: \"{data}\"")

    serialData += data

    if ";DONE" in serialData:
        # Robot is done moving
        robot_num = int(serialData[1])
        robots_still_working[robot_num] = False

    if "\n" in serialData:
        serialData = ""

    if robot_num in timeout_time and timeout_time[robot_num] < time.time():
        # Robot timed out
        print(f"ROBOT {robot_num} TIMED OUT...sending last command again!")
        ser.write(last_cmd.encode('utf-8'))

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
