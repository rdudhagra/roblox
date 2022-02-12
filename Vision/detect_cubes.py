import argparse
import cv2
import numpy as np
import pickle

from video_capture_threading import VideoCaptureThreading as VideoCapture

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

def detect_squares(threshold_img) -> "list(cv2.RotatedRect)":
    contours, _ = cv2.findContours(threshold_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    # Filter contours by area
    squares = [cv2.minAreaRect(c) for c in contours if cv2.contourArea(c) > 1000]
    # Filter squares by aspect ratio
    squares = [s for s in squares if 0.6 <= s[1][0] / s[1][1] <= 1.7]
    return squares

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
        hsv_img = preprocess_frame(frame)

        # Show frame
        for color in ["red", "green", "blue", "yellow", "purple", "orange"]:
            thresh_img = threshold_for_color(hsv_img, color)
            # keypoints = detect_blobs(thresh_img)

            # nice_debug_img = cv2.max(frame, np.repeat(
            #     thresh_img[:, :, np.newaxis], 3, axis=2))
            # for keypoint in keypoints:
            #     cv2.circle(final_img, (int(keypoint.pt[0]), int(keypoint.pt[1])),
            #             int(keypoint.size/2), (0, 0, 0), 5)

            # Find contours
            squares = detect_squares(thresh_img)
            for sq in squares:
                cv2.drawContours(frame, [np.int0(cv2.boxPoints(sq))], 0, (0, 0, 0), 3)
        cv2.imshow("frame", frame)

        # Check if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Stop the camera
    cap.stop()
    cv2.destroyAllWindows()
