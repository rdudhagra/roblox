import numpy as np

DIST_FROM_GRIPPER_TO_ROBOT = 53

def robot_acquisition_poses_from_cube_pose(cube_x, cube_y, cube_th):
    poses = []

    for angle_offset in [0, np.pi/2, np.pi, 3*np.pi/2]:
        th = cube_th + angle_offset

        # Normalize angle
        th = np.arctan2(np.sin(th), np.cos(th))

        x = cube_x - DIST_FROM_GRIPPER_TO_ROBOT * np.cos(th)
        y = cube_y - DIST_FROM_GRIPPER_TO_ROBOT * np.sin(th)
        poses.append(np.array([x, y, th]))

    return poses
