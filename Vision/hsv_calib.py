import argparse
import cv2
import numpy as np
import pickle

from video_capture_threading import VideoCaptureThreading

max_value = 255
max_value_H = 360 // 2
low_H = 0
low_S = 0
low_V = 0
high_H = max_value_H
high_S = max_value
high_V = max_value
window_detection_name = 'Object Detection'
low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'

def on_low_H_thresh_trackbar(val):
    global low_H
    global high_H
    low_H = val
    # low_H = min(high_H-1, low_H)
    cv2.setTrackbarPos(low_H_name, window_detection_name, low_H)


def on_high_H_thresh_trackbar(val):
    global low_H
    global high_H
    high_H = val
    # high_H = max(high_H, low_H+1)
    cv2.setTrackbarPos(high_H_name, window_detection_name, high_H)


def on_low_S_thresh_trackbar(val):
    global low_S
    global high_S
    low_S = val
    low_S = min(high_S-1, low_S)
    cv2.setTrackbarPos(low_S_name, window_detection_name, low_S)


def on_high_S_thresh_trackbar(val):
    global low_S
    global high_S
    high_S = val
    high_S = max(high_S, low_S+1)
    cv2.setTrackbarPos(high_S_name, window_detection_name, high_S)


def on_low_V_thresh_trackbar(val):
    global low_V
    global high_V
    low_V = val
    low_V = min(high_V-1, low_V)
    cv2.setTrackbarPos(low_V_name, window_detection_name, low_V)


def on_high_V_thresh_trackbar(val):
    global low_V
    global high_V
    high_V = val
    high_V = max(high_V, low_V+1)
    cv2.setTrackbarPos(high_V_name, window_detection_name, high_V)


if __name__ == "__main__":
    # Command line argument parsing
    parser = argparse.ArgumentParser()
    parser.add_argument("--cam_port", "-p", type=int, default=0, help="OpenCV camera port")
    parser.add_argument("--cap_width", "-x", type=int, default=3840, help="Camera capture width")
    parser.add_argument("--cap_height", "-y", type=int, default=2160, help="Camera capture height")
    parser.add_argument("--cap_fps", "-f", type=int, default=30, help="Camera capture FPS")
    parser.add_argument("--cam_calib", "-c", type=str, default="camera_calibration_data.pkl", help="Camera calibration")
    args = parser.parse_args()

    # Read frames from webcam
    cap = VideoCaptureThreading(
        port=args.cam_port,
        width=args.cap_width,
        height=args.cap_height,
        fps=args.cap_fps,
        calib=args.cam_calib,
    ).start()
    
    # Create calibration window
    cv2.namedWindow(window_detection_name)
    cv2.createTrackbar(low_H_name, window_detection_name, low_H,
                       max_value_H, on_low_H_thresh_trackbar)
    cv2.createTrackbar(high_H_name, window_detection_name, high_H,
                       max_value_H, on_high_H_thresh_trackbar)
    cv2.createTrackbar(low_S_name, window_detection_name, low_S,
                       max_value, on_low_S_thresh_trackbar)
    cv2.createTrackbar(high_S_name, window_detection_name, high_S,
                       max_value, on_high_S_thresh_trackbar)
    cv2.createTrackbar(low_V_name, window_detection_name, low_V,
                       max_value, on_low_V_thresh_trackbar)
    cv2.createTrackbar(high_V_name, window_detection_name, high_V,
                       max_value, on_high_V_thresh_trackbar)


    # Collect ROIs for all the block colors
    with open("cube_calibration_data.pkl", "rb") as f:
        [color_min_hsv, color_max_hsv] = pickle.load(f)

    for color in ["red", "orange", "yellow", "green", "blue",  "purple"]:
        if color in color_min_hsv and color in color_max_hsv:
            (low_H, low_S, low_V) = color_min_hsv[color]
            (high_H, high_S, high_V) = color_max_hsv[color]
        cv2.setTrackbarPos(low_H_name, window_detection_name, low_H)
        cv2.setTrackbarPos(high_H_name, window_detection_name, high_H)
        cv2.setTrackbarPos(low_S_name, window_detection_name, low_S)
        cv2.setTrackbarPos(high_S_name, window_detection_name, high_S)
        cv2.setTrackbarPos(low_V_name, window_detection_name, low_V)
        cv2.setTrackbarPos(high_V_name, window_detection_name, high_V)

        while True:
            # Read frame
            ret, frame = cap.read()

            if frame is None:
                break

            frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            if low_H > high_H:
                # Wrap around
                frame_threshold = cv2.bitwise_or(
                    cv2.inRange(frame_HSV, (low_H, low_S, low_V), (max_value_H, high_S, high_V)),
                    cv2.inRange(frame_HSV, (0, low_S, low_V), (high_H, high_S, high_V)),
                )
            else:
                frame_threshold = cv2.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))

            frame_combined = cv2.max(frame, np.repeat(frame_threshold[:, :, np.newaxis], 3, axis=2))
            frame_with_text = cv2.putText(frame_combined, f"Adjust sliders to select only {color} cubes:",
                (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (100, 100, 100), 5)

            cv2.imshow(window_detection_name, frame_with_text)

            key = cv2.waitKey(30)
            if key == ord('q') or key == 27:
                break

        color_min_hsv[color] = [low_H, low_S, low_V]
        color_max_hsv[color] = [high_H, high_S, high_V]


    # Save the mean and standard deviation values to a pickle file
    with open("cube_calibration_data.pkl", "wb") as f:
        pickle.dump([color_min_hsv, color_max_hsv], f)
