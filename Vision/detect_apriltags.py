import apriltag
import argparse
import cv2
import numpy as np

from utils import transform_point, transform_square, clamp_angle, box_angle
from video_capture_threading import VideoCaptureThreading as VideoCapture

# Initialize logger
if __name__ == "__main__":
    def log(message):
        print(message)
else:
    def log(message):
        pass

# Initialize apriltag detector
detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))

# Dimensions of field (in mm)
tag_dimx = 60
tag_dimy = 60
field_dimx = 840 # 11 x 3 = 33in
field_dimy = 650 # 8.5 x 3 = 25.5in

robot_ids = [8, 9]

def get_corners_to_detect(corner_type):
    # Returns: Dict from apriltag tag_id's => Position of tag center in world coordinates
    if corner_type == "cube":
        (x1, x2) = (3 * tag_dimx / 2, field_dimx - 3 * tag_dimx / 2)
        (y1, y2) = (tag_dimy / 2, field_dimy - tag_dimy / 2)
        corners_cube = { # Same height as cube
            0: np.array([x1, y1]),
            1: np.array([x2, y1]),
            2: np.array([x1, y2]),
            3: np.array([x2, y2]),
        }
        return corners_cube

    elif corner_type == "robot":
        (x1, x2) = (tag_dimx / 2, field_dimx - tag_dimx / 2)
        (y1, y2) = (tag_dimy / 2, field_dimy - tag_dimy / 2)
        corners_robot = { # Same height as robot
            4: np.array([x1, y1]),
            5: np.array([x2, y1]),
            6: np.array([x1, y2]),
            7: np.array([x2, y2]),
        }
        return corners_robot

    else:
        return None

def detect_apriltags(frame):
    # Detects apriltags in current frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(gray)
    return tags

def get_robot_poses(tags, img2world_robot):
    # Returns position and orientation of robot in world frame
    robots = {}
    if img2world_robot is None:
        return robots

    for tag in tags:
        if tag.tag_id in robot_ids:
            corners = transform_square(img2world_robot, tag.corners)
            center = np.mean(corners, axis=0)
            th = box_angle(corners)
            robots[tag.tag_id] = (center, th)
            log(f"Robot {tag.tag_id}: pos={center}, th={th * 180 / np.pi}")
    return robots

def get_img2world_transform(tags, corner_type):
    # tags: Output of apriltag detection algorithm
    # corners_to_detect: A dict of (tag_id => tag center positions in world)
    # Extract the relevant corners from the list of apriltag detections and
    # use linear least squares to fit an image-to-world affine transform to the tags
    # Returns: The img2world transform on success, or None on failure
    
    corners_to_detect = get_corners_to_detect(corner_type)

    # Find tag centers in image
    corners = {}
    for tag in tags:
        if tag.tag_id in corners_to_detect.keys():
            corners[tag.tag_id] = tag.center

    # Abort if four corners are not visible
    for tag_id in corners_to_detect.keys():
        if tag_id not in corners:
            return None

    # Determine source (image) and destination (world) coordinates
    src = []
    dst = []
    for (tag_id, (xw, yw)) in corners_to_detect.items():
        (xi, yi) = corners[tag_id]
        src.append([xi, yi])
        dst.append([xw, yw])
    src = np.array(src, dtype=np.float32)
    dst = np.array(dst, dtype=np.float32)

    # Compute perspective transform
    img2world = cv2.getPerspectiveTransform(src, dst)
    return img2world

def test_img2world_transform(tags, img2world, corner_type):
    # Tests the accuracy of the img2world transform
    # corners_to_detect: Dict mapping tag_id => world coordinates of tag

    corners_to_detect = get_corners_to_detect(corner_type)

    # Find tag centers in image
    corners = {}
    for tag in tags:
        if tag.tag_id in corners_to_detect.keys():
            corners[tag.tag_id] = tag.center

    # Abort if four corners are not visible
    for tag_id in corners_to_detect.keys():
        if tag_id not in corners:
            return None

    # Find error between actual and expected corner positions
    error = 0
    for (tag_id, img_pos) in corners.items():
        world_pos_exp = corners_to_detect[tag_id]
        world_pos_act = transform_point(img2world, img_pos)
        error += np.linalg.norm(world_pos_exp - world_pos_act)
        log(f"Tag {tag_id}: Expected {tuple(world_pos_exp)}, Actual {tuple(world_pos_act)}")
    error /= len(corners)

    log(f"Error: {error}")
    return error


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cam_port", "-p", type=int, default=0, help="OpenCV camera port")
    parser.add_argument("--cap_width", "-x", type=int, default=3840, help="Camera capture width")
    parser.add_argument("--cap_height", "-y", type=int, default=2160, help="Camera capture height")
    parser.add_argument("--cap_fps", "-f", type=int, default=30, help="Camera capture FPS")
    parser.add_argument("--cam_calib", "-c", type=str, default="camera_calibration_data.pkl", help="Camera calibration")
    parser.add_argument("--use_calib", "-s", action="store_true")
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
        cv2.imshow("frame", frame)

        # Detect apriltags
        tags = detect_apriltags(frame)

        # Find img2world transform for cube height corners
        img2world_cube = get_img2world_transform(tags, "cube")
        print("Testing cube height corners")
        test_img2world_transform(tags, img2world_cube, "cube")

        # Find img2world transform for robot height corners
        img2world_robot = get_img2world_transform(tags, "robot")
        print("Testing robot height corners")
        test_img2world_transform(tags, img2world_robot, "robot")

        # Find robot poses
        # get_robot_poses(tags, robot_ids, img2world_robot)
        get_robot_poses(tags, img2world_robot)

        # Check if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera
    cap.stop()
    cv2.destroyAllWindows()
