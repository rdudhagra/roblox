import argparse
import cv2
import numpy as np
import time

from detect_apriltags import detect_apriltags, get_img2world_transform, get_robot_poses
from detect_cubes import detect_cubes
from video_capture_threading import VideoCaptureThreading as VideoCapture
from pose_utils import robot_acquisition_poses_from_cube_pose
from robot_interface import *

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

    print("Sleeping for a bit...")
    time.sleep(5)
    print("ok imma start now")

    # Show frames until 'q' is pressed
    while True:
        # Read frame
        ret, frame = cap.read_calib() if args.use_calib else cap.read()

        # Detect apriltags and find img2world transform for cube height corners
        tags = detect_apriltags(frame)

        # Find img2world transforms
        img2world_cube = get_img2world_transform(tags, "cube")
        img2world_robot = get_img2world_transform(tags, "robot")

        # Find robot poses
        robot_poses = get_robot_poses(tags, img2world_robot)

        # Detect cubes
        (all_cubes, all_thresh, frame) = detect_cubes(frame, img2world_cube)
        cv2.imshow("frame", frame)

        for (color, cubes) in all_cubes.items():
            print(f"{color} cubes: {' / '.join([repr(((x, y), th * 180 / np.pi)) for ((x, y), th) in cubes])}")


        # Drive to cube-proof of concept
        if len(all_cubes["green"]) == 0:
            continue
        first_cube = all_cubes["green"][0]
        rob_pose = robot_poses[9]
        # Tell robot where it is
        set_pose(1, rob_pose[0][0], rob_pose[0][1], rob_pose[1])
        print(rob_pose)
        time.sleep(0.1)

        acq_poses = robot_acquisition_poses_from_cube_pose(first_cube[0][0], first_cube[0][1], first_cube[1])
        selected_pose = acq_poses[np.argmin([np.linalg.norm(p[0:2] - rob_pose[0]) for p in acq_poses])]
        print(selected_pose)
        drive(1, *selected_pose)
        wait(1)
        pick(1)
        wait(1)
        drive(1, selected_pose[0] + 100, selected_pose[1], selected_pose[2])
        wait(1)
        place(1)
        wait(1)

        exit(0)

        # Check if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera
    cap.stop()
    cv2.destroyAllWindows()
