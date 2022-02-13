import argparse
import cv2
import numpy as np

from detect_apriltags import detect_apriltags, get_img2world_transform, get_robot_poses, field_dimx, field_dimy
from detect_cubes import detect_cubes
from utils import transform_point
from video_capture_threading import VideoCaptureThreading as VideoCapture

# Specify radius of each obstacle and how far to stay away from obstacles
cube_radius = 18  # Radius of cube (mm)
robot_radius = 80 # Radius of robot (mm)
avoid_dist = 30   # How far to stay away from each object's bounding box (mm)

def circle_line_intersection(segment, circle):
    # Returns the number of circle-segment intersections
    # segment: ((ax, ay), (bx, by))
    # circle: (cx, cy, r)
    ((ax, ay), (bx, by)) = segment
    (cx, cy, r) = circle
    a = np.array([ax, ay])
    b = np.array([bx, by])
    c = np.array([cx, cy])

    # Compute point D by taking projection of AC onto AB then adding the offset of A
    ac = c - a
    ab = b - a
    d = np.dot(ac, ab) / np.linalg.norm(ab) + a
    ad = d - a

    # D might not be on AB, so calculate k of D down AB (solve AD = k * AB)
    # Choose to use larger component to reduce chance of dividing by zero
    k = ad[0] / ab[0] if np.abs(ab[0]) > np.abs(ab[1]) else ad[1] / ab[1]

    # Check if D is off either end of line segment
    if k <= 0.0:
        dist = np.linalg.norm(c - a)
        return 1 if dist <= r else 0
    elif k >= 1.0:
        dist = np.linalg.norm(c - b)
        return 1 if dist <= r else 0
    else:
        dist = np.linalg.norm(c - d)
        return 2 if dist <= r else 0

def compute_obstacle_circles(robot_idx, robots, all_cubes):
    # Return a list of obstacles to avoid, as circles
    circles = []
    for (idx, robot) in robots.items():
        if idx != robot_idx:
            ((x, y), th) = robot
            circles.append((x, y, robot_radius + avoid_dist))

    for (color, cubes) in all_cubes.items():
        for cube in cubes:
            ((x, y), th) = cube
            circles.append((x, y, cube_radius + avoid_dist))

    return circles

def compute_reachable_cubes(robot_pos, all_cubes):
    # Filter the list of cubes to those reachable in a straight-line path from the current robot
    # Returns a list of reachable cubes in the same format as all_cubes
    
    (rx, ry) = robot_pos

    # For each cube, compute whether that cube is reachable
    reachable_cubes = {}
    for (color, cubes) in all_cubes.items():
        reachable_cubes[color] = []
        for cube in cubes:
            ((cx, cy), _) = cube

            # Loop through all other cubes to determine reachability
            is_reachable = True
            for (_, cubes_oth) in all_cubes.items():
                for cube_oth in cubes_oth:
                    ((ox, oy), _) = cube_oth
                    # Cube is not reachable if it intersects with some other circle
                    if circle_line_intersection(((rx, ry), (ox, oy)), (cx, cy, cube_radius + avoid_dist)) > 0 and \
                            (ox, oy) != (cx, cy):
                        is_reachable = False
                        break
                if not is_reachable:
                    break
            if is_reachable:
                reachable_cubes[color].append(cube)

    return reachable_cubes

def show_obstacles(circles, frame, world2img_cube, world2img_robot):
    # Draw the given circles as obstacles in the frame
    # Use the world2img_cube transform for cube-sized obstacles and
    # use the world2img_robot transform for robot-sized obstacles
    if world2img_cube is None or world2img_robot is None:
        return frame

    frame_ = frame.copy()
    for (x, y, r) in circles:
        if r == robot_radius + avoid_dist:
            (x_img, y_img) = transform_point(world2img_robot, (x, y))
        elif r == cube_radius + avoid_dist:
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

        # Detect cubes and compute reachable cubes
        (all_cubes, all_thresh, frame) = detect_cubes(frame, img2world_cube)
        reachable_cubes = compute_reachable_cubes(robots[8][0], all_cubes) if 8 in robots else {}

        # Show all cube hitboxes
        circles = compute_obstacle_circles(8, robots, all_cubes)
        frame = show_obstacles(circles, frame, world2img_cube, world2img_robot)
        
        # Show reachable cubes
        for (color, cubes) in reachable_cubes.items():
            for ((cx, cy), _) in cubes:
                (cx_img, cy_img) = transform_point(world2img_cube, (cx, cy))
                cv2.circle(frame, (int(cx_img), int(cy_img)), cube_radius + avoid_dist, (255, 0, 0), 3)
        cv2.imshow("frame", frame)

        # Check if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera
    cap.stop()
    cv2.destroyAllWindows()
