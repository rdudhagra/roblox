import cv2
import numpy as np
import pickle

# Read cube calibration data
with open("cube_calibration_data.pkl", "rb") as f:
    [cube_color_mins, cube_color_maxs] = pickle.load(f)


def threshold_for_color(hsv_img, color):
    (low_H, low_S, low_V) = cube_color_mins[color]
    (high_H, high_S, high_V) = cube_color_maxs[color]
    if low_H > high_H:
        # Wrap around
        frame_threshold = cv2.bitwise_or(
            cv2.inRange(hsv_img, (low_H, low_S, low_V),
                        (360, high_S, high_V)),
            cv2.inRange(hsv_img, (0, low_S, low_V),
                        (high_H, high_S, high_V)),
        )
    else:
        frame_threshold = cv2.inRange(
            hsv_img, (low_H, low_S, low_V), (high_H, high_S, high_V))
    return frame_threshold


if __name__ == "__main__":
    # Read frames from webcam
    cap = cv2.VideoCapture(2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FPS, 30)

    # Create a window
    cv2.namedWindow("frame", cv2.WINDOW_NORMAL)

    # Show frames until 'q' is pressed
    while True:
        # Read frame
        ret, frame = cap.read()

        # Convert to HSV
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Show frame
        cv2.imshow("frame", frame)
        cv2.imshow("frame", cv2.max(frame, np.repeat(
            threshold_for_color(hsv_img, "red")[:, :, np.newaxis], 3, axis=2)))

        # Check if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera
    cap.release()
