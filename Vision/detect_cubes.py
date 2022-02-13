import argparse
import cv2
import numpy as np
import pickle

from utils import transform_point, transform_square, clamp_angle, box_angle
from video_capture_threading import VideoCaptureThreading as VideoCapture

# Initialize logger
if __name__ == "__main__":
    def log(message):
        print(message)
else:
    def log(message):
        pass

# Read cube calibration data
with open("cube_calibration_data.pkl", "rb") as f:
    [cube_color_mins, cube_color_maxs] = pickle.load(f)

def preprocess_frame(frame):
    # Convert to HSV
    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Sharpen
    frame_sharpen = cv2.GaussianBlur(frame, (0, 0), 1)
    frame_sharpen = cv2.addWeighted(frame, 1.5, frame_sharpen, -0.5, 0)

    return hsv_img

def threshold_for_color(hsv_img, color):
    (low_H, low_S, low_V) = cube_color_mins[color]
    (high_H, high_S, high_V) = cube_color_maxs[color]
    if low_H > high_H:
        # Wrap around
        frame_threshold = cv2.bitwise_or(
            cv2.inRange(hsv_img, (low_H, low_S, low_V), (360, high_S, high_V)),
            cv2.inRange(hsv_img, (0, low_S, low_V), (high_H, high_S, high_V)),
        )
    else:
        frame_threshold = cv2.inRange(hsv_img, (low_H, low_S, low_V), (high_H, high_S, high_V))

    morph_ellipse = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    frame_threshold = cv2.morphologyEx(frame_threshold, cv2.MORPH_OPEN, morph_ellipse)
    return frame_threshold

def detect_squares(threshold_img):
    # Returns a list of cv2.RotatedRect

    # Use OpenCV to find contours
    contours, _ = cv2.findContours(threshold_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    # Filter squares by area and aspect ratio
    squares = [cv2.minAreaRect(c) for c in contours if cv2.contourArea(c) > 50]
    squares = [s for s in squares if 0.6 <= s[1][0] / s[1][1] <= 1.7]
    squares = [cv2.boxPoints(sq) for sq in squares]
    
    return squares

def get_cube_poses(squares, img2world_cube):
    # Returns position and orientation of cubes in world frame
    cubes = []
    if img2world_cube is None:
        return cubes

    for sq in squares:
        corners = transform_square(img2world_cube, sq)
        center = np.mean(corners, axis=0)
        th = box_angle(corners, np.pi/2)
        cubes.append((center, th))
        log(f"Square: pos={center}, th={th * 180 / np.pi}")
    return cubes

def draw_squares(frame, squares):
    for sq in squares:
        cv2.drawContours(frame, [np.int0(sq)], 0, (0, 0, 0), 3)

def detect_cubes(frame, img2world_cube):
    # Detect cubes in the given frame
    # Returns: (all_cubes, all_thresh, frame)
    # all_cubes: Dict from color to list of ((x, y), th) of cubes at that index
    # all_thresh: Binary image of all drivable area
    # frame: Input frame with cube boundaries drawn on

    # Preprocess image
    frame_ = frame.copy()
    hsv_img = preprocess_frame(frame)

    all_thresh = None
    all_cubes = {}
    for color in ["red", "green", "blue", "yellow", "purple", "orange"]:
        thresh_img = threshold_for_color(hsv_img, color)
        all_thresh = thresh_img if all_thresh is None else np.bitwise_or(all_thresh, thresh_img)

        squares = detect_squares(thresh_img)
        cubes = get_cube_poses(squares, img2world_cube)
        all_cubes[color] = cubes

        draw_squares(frame_, squares)

    return (all_cubes, all_thresh, frame_)

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

        # Show frame
        for color in ["red", "green", "blue", "yellow", "purple", "orange"]:
            thresh_img = threshold_for_color(hsv_img, color)

            # Find contours
            squares = detect_squares(thresh_img)
            draw_squares(frame, squares)

        cv2.imshow("frame", frame)

        # Check if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Stop the camera
    cap.stop()
    cv2.destroyAllWindows()
