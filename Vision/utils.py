import numpy as np
from enum import Enum

class CUBE_PROGRESS(Enum):
    HAS_NOT_STARTED = 0
    GOING_TO_CUBE = 1
    PICKING_UP_CUBE = 2
    BACKING_UP_PICK = 3
    GOING_TO_DROP_LOCATION = 4
    DROPPING_CUBE = 5
    BACKING_UP_DROP = 6
    DONE = 7

    def __eq__(self, other):
        return self.value == other  

    def __int__(self):
        return self.value

def transform_point(transform, point):
    # Apply a 3x3 perspective transform to a 2D point (x, y)
    x = np.array([point[0], point[1], 1.])
    out = transform @ x
    return np.array([out[0] / out[2], out[1] / out[2]])

def transform_square(transform, square):
    return np.array([transform_point(transform, p) for p in square])

def clamp_angle(theta, limit=2*np.pi):
    while theta < 0:
        theta += limit
    while theta >= limit:
        theta -= limit
    return theta

def box_angle(box, limit=2*np.pi):
    fwd = box[0] - box[3]
    th = clamp_angle(np.arctan2(fwd[1], fwd[0]), limit)
    return th

def rob_to_april_tag(robot_num):
    return robot_num + 8
