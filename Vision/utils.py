import numpy as np

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
