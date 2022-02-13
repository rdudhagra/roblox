from path_planning import compute_reachable_cubes
from utils import CUBE_PROGRESS, rob_to_april_tag
from image import *
from robot_interface import *
from pose_utils import robot_acquisition_poses_from_cube_pose
from video_capture_threading import VideoCaptureThreading as VideoCapture
from detect_cubes import detect_cubes
from detect_apriltags import detect_apriltags, get_img2world_transform, get_robot_poses
import argparse
import cv2
import numpy as np
import time

ROBOTS = [0]


if __name__ == "__main__":
    # Command line argument parsing
    parser = argparse.ArgumentParser()
    parser.add_argument("--cam_port", "-p", type=int,
                        default=3, help="OpenCV camera port")
    parser.add_argument("--cap_width", "-x", type=int,
                        default=3840, help="Camera capture width")
    parser.add_argument("--cap_height", "-y", type=int,
                        default=2160, help="Camera capture height")
    parser.add_argument("--cap_fps", "-f", type=int,
                        default=30, help="Camera capture FPS")
    parser.add_argument("--cam_calib", "-c", type=str,
                        default="camera_calibration_data.pkl", help="Camera calibration")
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
    time.sleep(3)
    print("just a lil bit longer...")
    time.sleep(3)
    print("dw im still here...")
    time.sleep(4)
    print("ok imma start now :)")

    image = np.array(FLIP_SMILE_IMAGE)
    image_progress = np.zeros_like(image)
    robot_progress: "dict(int, CUBE_PROGRESS)" = {}
    for robot in ROBOTS:
        robot_progress[robot] = CUBE_PROGRESS.HAS_NOT_STARTED

    robot_drop_locations = {}
    robot_pick_locations = {}

    TASK_DONE = False

    # Show frames until 'q' is pressed
    while True:
        # Read frame
        ret, frame = cap.read_calib() if args.use_calib else cap.read()
        cv2.imshow("frame", frame)
        cv2.waitKey(1)

        # Detect apriltags and find img2world transform for cube height corners
        tags = detect_apriltags(frame)

        # Find img2world transforms
        img2world_cube = get_img2world_transform(tags, "cube")
        img2world_robot = get_img2world_transform(tags, "robot")

        if img2world_cube is None or img2world_robot is None:
            time.sleep(0.03)
            print("ERROR: Cannot find April tags. Skipping frame.")
            continue

        # Find robot poses
        robot_poses = get_robot_poses(tags, img2world_robot)

        # Detect cubes
        (all_cubes, all_thresh, frame) = detect_cubes(frame, img2world_cube)

        # Do task based on robot status
        for robot in ROBOTS:
            if isRobotRunning(robot):
                time.sleep(0.03)
                continue

            if rob_to_april_tag(robot) not in robot_poses:
                time.sleep(0.03)
                continue

            rob_pose = robot_poses[rob_to_april_tag(robot)]
            # refine_pose(robot, rob_pose[0][0], rob_pose[0][1], rob_pose[1])

            if robot_progress[robot] == CUBE_PROGRESS.HAS_NOT_STARTED:
                # Update robot pose from computer vision
                set_pose(robot, rob_pose[0][0], rob_pose[0][1], rob_pose[1])

                # Find a cube to pick up
                cubes_needed_now = cubes_needed(image, image_progress)
                if cubes_needed_now is None or len(cubes_needed_now) == 0:
                    # We're done!
                    TASK_DONE = True

                else:
                    # First cube for now, this can be made better later
                    cube_color, image_ind, cube_pos = cubes_needed_now[0]
                    image_progress[image_ind[0], image_ind[1]] = 1

                    cube_pos[2] = np.pi # Force acquisition from the right side
                    robot_drop_locations[robot] = cube_pos

                    reachable_cubes = compute_reachable_cubes(
                        rob_pose[0], all_cubes)[cube_color_var_to_str(cube_color)]
                    if len(reachable_cubes) == 0:
                        print("No reachable cubes")
                        continue
                    ((cube_x, cube_y), cube_th) = reachable_cubes[0]
                    acq_poses = robot_acquisition_poses_from_cube_pose(
                        cube_x, cube_y, cube_th)
                    selected_pose = acq_poses[np.argmin(
                        [np.linalg.norm(p[0:2] - rob_pose[0]) for p in acq_poses])]
                    robot_pick_locations[robot] = selected_pose

                    print(
                        f"Picking up a {cube_color_var_to_str(cube_color)} cube from {selected_pose} to {cube_pos}")

                    pre_select_pose = list(selected_pose)
                    pre_select_pose[0] -= np.cos(pre_select_pose[2]) * 50
                    pre_select_pose[1] -= np.sin(pre_select_pose[2]) * 50
                    drive(robot, *pre_select_pose)

                    robot_progress[robot] = CUBE_PROGRESS.GOING_TO_CUBE

            elif robot_progress[robot] == CUBE_PROGRESS.GOING_TO_CUBE:
                set_pose(robot, rob_pose[0][0], rob_pose[0][1], rob_pose[1])
                drive(robot, *robot_pick_locations[robot])

                robot_progress[robot] = CUBE_PROGRESS.REFINING_PICK_POS

            elif robot_progress[robot] == CUBE_PROGRESS.REFINING_PICK_POS:
                pick(robot)

                robot_progress[robot] = CUBE_PROGRESS.PICKING_UP_CUBE

            elif robot_progress[robot] == CUBE_PROGRESS.PICKING_UP_CUBE:
                reset_pose = [rob_pose[0][0], rob_pose[0][1], rob_pose[1]]
                reset_pose[0] -= 100
                # Turn around to face the image being constructed
                reset_pose[2] = np.pi
                reverse(robot, *reset_pose)

                robot_progress[robot] = CUBE_PROGRESS.BACKING_UP_PICK

            elif robot_progress[robot] == CUBE_PROGRESS.BACKING_UP_PICK:
                set_pose(robot, rob_pose[0][0], rob_pose[0][1], rob_pose[1])
                pre_select_pose = list(robot_drop_locations[robot])
                pre_select_pose[0] -= np.cos(pre_select_pose[2]) * 50
                pre_select_pose[1] -= np.sin(pre_select_pose[2]) * 50
                drive(robot, *pre_select_pose)

                robot_progress[robot] = CUBE_PROGRESS.GOING_TO_DROP_LOCATION

            elif robot_progress[robot] == CUBE_PROGRESS.GOING_TO_DROP_LOCATION:
                set_pose(robot, rob_pose[0][0], rob_pose[0][1], rob_pose[1])
                drive(robot, *robot_drop_locations[robot])

                robot_progress[robot] = CUBE_PROGRESS.REFINING_DROP_POS

            elif robot_progress[robot] == CUBE_PROGRESS.REFINING_DROP_POS:
                place(robot)

                robot_progress[robot] = CUBE_PROGRESS.DROPPING_CUBE

            elif robot_progress[robot] == CUBE_PROGRESS.DROPPING_CUBE:
                set_pose(robot, rob_pose[0][0], rob_pose[0][1], rob_pose[1])
                reset_pose = [rob_pose[0][0], rob_pose[0][1], rob_pose[1]]
                reset_pose[0] += 100
                reset_pose[2] = 0  # Turn around to face the unplaced cubes
                reverse(robot, *reset_pose)

                robot_progress[robot] = CUBE_PROGRESS.BACKING_UP_DROP

            elif robot_progress[robot] == CUBE_PROGRESS.BACKING_UP_DROP:
                # Nothing to do here!

                robot_progress[robot] = CUBE_PROGRESS.DONE

            elif robot_progress[robot] == CUBE_PROGRESS.DONE:
                # Nothing to do here!

                robot_progress[robot] = CUBE_PROGRESS.HAS_NOT_STARTED

            print(f"Robot {robot}: {robot_progress[robot]}")

        # Check if we are done with the entire image
        if TASK_DONE:
            if np.all(robot_progress.values() == CUBE_PROGRESS.DONE):
                break

    # Release the camera
    cap.stop()
    cv2.destroyAllWindows()
