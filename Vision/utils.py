import numpy as np

def transform_point(transform, point):
    # Apply a 3x3 perspective transform to a 2D point (x, y)
    x = np.array([point[0], point[1], 1.])
    out = transform @ x
    return np.array([out[0] / out[2], out[1] / out[2]])

def clamp_angle(theta):
    while theta < 0:
        theta += 2 * np.pi
    while theta >= 2 * np.pi:
        theta -= 2 * np.pi
    return theta
