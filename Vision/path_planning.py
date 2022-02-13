import argparse
import cv2
import numpy as np

from detect_apriltags import detect_apriltags, get_img2world_transform, get_robot_poses
from detect_cubes import detect_cubes
from utils import transform_point
from video_capture_threading import VideoCaptureThreading as VideoCapture

cube_radius = 18
robot_radius = 50

def compute_path(start_pose, end_pose, robot_idx, robots, all_cubes):
    # Compute a path from the start to the end, avoiding cubes and other robots

    # Find a list of all obstacles to avoid, as circles
    circles = []
    for (idx, robot) in robots.items():
        if idx != robot_idx:
            ((x, y), th) = robot
            circles.append((x, y, robot_radius))

    for (color, cubes) in all_cubes.items():
        for cube in cubes:
            ((x, y), th) = cube
            circles.append((x, y, cube_radius))

    return circles

def show_obstacles(circles, frame, world2img_cube, world2img_robot):
    # Draw the given circles as obstacles in the frame
    # Use the world2img_cube transform for cube-sized obstacles and
    # use the world2img_robot transform for robot-sized obstacles
    if world2img_cube is None or world2img_robot is None:
        return frame

    frame_ = frame.copy()
    for (x, y, r) in circles:
        if r == robot_radius:
            (x_img, y_img) = transform_point(world2img_robot, (x, y))
        elif r == cube_radius:
            (x_img, y_img) = transform_point(world2img_cube, (x, y))
        else:
            continue

        cv2.circle(frame_, (int(x_img), int(y_img)), r, (0, 0, 0), 3)

    return frame_

if __name__ == "__main__":
    # Command line argument parsing
    parser = argparse.ArgumentParser()
    parser.add_argument("--cam_port", "-p", type=int, default=0, help="OpenCV camera port")
    parser.add_argument("--cap_width", "-x", type=int, default=3840, help="Camera capture width")
    parser.add_argument("--cap_height", "-y", type=int, default=2160, help="Camera capture height")
    parser.add_argument("--cap_fps", "-f", type=int, default=30, help="Camera capture FPS")
    parser.add_argument("--cam_calib", "-c", type=str, default="camera_calibration_data.pkl", help="Camera calibration")
    parser.add_argument("--use_calib", "-u", action="store_true")
    args = parser.parse_args()

    # Read frames from webcam
    cap = VideoCapture(
        port=args.cam_port,
        width=args.cap_width,
        height=args.cap_height,
        fps=args.cap_fps,
        calib=args.cam_calib,
    ).start()

    # Create a window
    cv2.namedWindow("frame", cv2.WINDOW_NORMAL)

    # Show frames until 'q' is pressed
    while True:
        # Read frame
        ret, frame = cap.read_calib() if args.use_calib else cap.read()

        # Detect apriltags
        tags = detect_apriltags(frame)

        # Find img2world transforms and their inverses
        img2world_cube = get_img2world_transform(tags, "cube")
        img2world_robot = get_img2world_transform(tags, "robot")
        world2img_cube = np.linalg.inv(img2world_cube) if img2world_cube is not None else None
        world2img_robot = np.linalg.inv(img2world_robot) if img2world_robot is not None else None

        # Find robot poses
        robots = get_robot_poses(tags, img2world_robot)

        # Detect cubes
        (all_cubes, all_thresh, frame) = detect_cubes(frame, img2world_cube)

        # Compute path
        circles = compute_path((0, 0), (600, 600), 8, robots, all_cubes)
        frame = show_obstacles(circles, frame, world2img_cube, world2img_robot)
        cv2.imshow("frame", frame)

        # Check if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera
    cap.stop()
    cv2.destroyAllWindows()
