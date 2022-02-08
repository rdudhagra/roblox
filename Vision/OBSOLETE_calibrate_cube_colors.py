######## DOES NOT WORK #########
### USE HSV_CALIB.PY INSTEAD ###

import cv2
import numpy as np
import pickle
import time

if __name__ == "__main__":
    # Read frame from webcam
    cap = cv2.VideoCapture(2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
    cap.set(cv2.CAP_PROP_FPS, 30)

    # Flush some frames (for 5 seconds) so the camera can adjust its exposure/etc
    t = time.time()
    while time.time() - t < 5:
        cap.read()

    ret, frame = cap.read()
    cap.release()

    # Create HSV image
    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a window
    cv2.namedWindow("frame", cv2.WINDOW_NORMAL)

    # Collect ROIs for all the block colors
    roi_mapping = {}
    for color in ["red", "green", "blue", "orange", "yellow", "purple"]:
        # Copy frame
        frame_copy = frame.copy()

        # Show frame
        frame_with_text = cv2.putText(frame_copy, f"Select ROI corresponding to a {color} cube (click and drag):", (
            10, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (100, 100, 100), 5)

        # Query for ROI
        roi_mapping[color] = cv2.selectROI("frame", frame_with_text, False, False)

    # Calculate the mean and standard deviation HSV values for each ROI
    roi_min = {}
    roi_max = {}

    for color, roi in roi_mapping.items():
        roi_img = hsv_img[roi[1]:roi[1] + roi[3], roi[0]:roi[0] + roi[2]]
        roi_min[color] = np.min(roi_img, axis=(0, 1))
        roi_max[color] = np.max(roi_img, axis=(0, 1))

    # Save the mean and standard deviation values to a pickle file
    with open("cube_calibration_data.pkl", "wb") as f:
        pickle.dump([roi_min, roi_max], f)
