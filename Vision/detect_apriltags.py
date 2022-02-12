import apriltag
import argparse
import cv2
import numpy as np

from video_capture_threading import VideoCaptureThreading as VideoCapture

# Initialize apriltag detector
detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))

# Dimensions of field (in mm)
tag_dimx = 60
tag_dimy = 60
field_dimx = 840 # 11 x 3 = 33in
field_dimy = 650 # 8.5 x 3 = 25.5in

# Apriltag tag_id's to recognize => Position of tag center in world coordinates
(x1, x2) = (3 * tag_dimx / 2, field_dimx - 3 * tag_dimx / 2)
(y1, y2) = (tag_dimy / 2, field_dimy - tag_dimy / 2)
corners_cube = { # Same height as cube
    0: (x1, y1),
    1: (x2, y1),
    2: (x1, y2),
    3: (x2, y2),
}

(x1, x2) = (tag_dimx / 2, field_dimx - tag_dimx / 2)
(y1, y2) = (tag_dimy / 2, field_dimy - tag_dimy / 2)
corners_robot = { # Same height as robot
    4: (x1, y1),
    5: (x2, y1),
    6: (x1, y2),
    7: (x2, y2),
}

def apply_transform(transform, point):
    # Apply a 3x3 homogeneous transform to a 2D point (x,y)
    x = np.array([point[0], point[1], 1])
    out = transform @ x
    return out[:2]

def detect_apriltags(frame):
    # Detects apriltags in current frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(gray)
    return tags

def find_apriltag_corners(tags, ids_to_detect):
    # ids_to_detect: List of apriltag tag_id's to detect
    # Returns: On success, a dict mapping tag_id => image coordinates of tag center
    #          On failure, returns None

    return corners

def get_img2world_transform(tags, corners_to_detect):
    # tags: Output of apriltag detection algorithm
    # corners_to_detect: A dict of (tag_id => tag center positions in world)
    # Extract the relevant corners from the list of apriltag detections and
    # use linear least squares to fit an image-to-world affine transform to the tags
    # Returns: The img2world transform on success, or None on failure
    
    # Find tag centers in image
    corners = {}
    for tag in tags:
        if tag.tag_id in corners_to_detect.keys():
            corners[tag.tag_id] = tag.center

    # Abort if four corners are not visible
    for tag_id in ids_to_detect:
        if tag_id not in corners:
            return None

    # Build homography matrices
    # Reference: http://cs.cmu.edu/~16385/lectures/lecture7.pdf, slide 79
    b = []
    A = []
    for (tag_id, (xw, yw)) in corners_to_detect:
        (xi, yi) = corners[tag_id]

        b.append(np.array([[xw], [yw]]))
        A.append(np.array([[xi, yi, 1., 0., 0., 0.],
                           [0., 0., 0., xi, yi, 1.]]))

    b = np.concatenate(b, dim=0)
    A = np.concatenate(A, dim=0)

    # Solve linear least squares
    AT = np.transpose(A)
    x = np.linalg.inv(AT @ A) @ AT @ b

    # Construct 3x3 matrix from solution
    img2world = np.array([
        [x[0], x[1], x[2]],
        [x[3], x[4], x[5]],
        [0, 0, 1],
    ])
    return img2world

def test_img2world_transform(tags, img2world, corners_to_detect):
    # Tests the accuracy of the img2world transform
    # corners_to_detect: Dict mapping tag_id => world coordinates of tag

    # Find tag centers in image
    corners = {}
    for tag in tags:
        if tag.tag_id in corners_to_detect.keys():
            corners[tag.tag_id] = tag.center

    # Abort if four corners are not visible
    for tag_id in ids_to_detect:
        if tag_id not in corners:
            return None

    # Find error between actual and expected corner positions
    error = 0
    for (tag_id, img_pos) in corners.items():
        world_pos_exp = corners_to_detect[tag_id]
        world_pos_act = apply_transform(img2world, img_pos)
        error += np.linalg.norm(world_pos_exp - world_pos_act)
        print(f"Tag {tag_id}: Expected {tuple(world_pos_exp)}, Actual {tuple(world_pos_act)}")
    error /= len(corners)

    print(f"Error: {error / len(corners)}")
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
        img2world_cube = get_img2world_transform(tags, corners_cube)
        print("Testing cube height corners")
        test_img2world_transform(tags, img2world_cube, corners_cube)

        # Find img2world transform for robot height corners
        img2world_robot = get_img2world_transform(tags, corners_robot)
        print("Testing robot height corners")
        test_img2world_transform(tags, img2world_robot, corners_robot)

        # Check if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera
    cap.stop()
    cv2.destroyAllWindows()
