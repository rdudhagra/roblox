import cv2
import numpy as np
import pickle

# Setup SimpleBlobDetector parameters.
blobDetectorParams = cv2.SimpleBlobDetector_Params()

# Change thresholds
blobDetectorParams.minThreshold = 127
blobDetectorParams.maxThreshold = 129
blobDetectorParams.thresholdStep = 1

# Filter by Area.
blobDetectorParams.filterByArea = True
blobDetectorParams.minArea = 5000
blobDetectorParams.maxArea = 500000

# Filter by Circularity
blobDetectorParams.filterByCircularity = False
# blobDetectorParams.minCircularity = 0.2
# blobDetectorParams.maxCircularity = 1.0

# Filter by Convexity
blobDetectorParams.filterByConvexity = True
blobDetectorParams.minConvexity = 0.87
blobDetectorParams.maxConvexity = 1.0

# Filter by Inertia
blobDetectorParams.filterByInertia = True
blobDetectorParams.minInertiaRatio = 0.01
blobDetectorParams.maxInertiaRatio = 1

# Filter by Color
blobDetectorParams.filterByColor = True
blobDetectorParams.blobColor = 255

# Create a detector with the parameters
detector = cv2.SimpleBlobDetector_create(blobDetectorParams)


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

    frame_threshold = cv2.morphologyEx(
        frame_threshold, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    return frame_threshold


def detect_blobs(threshold_img):
    # Detect blobs.
    keypoints = detector.detect(threshold_img)
    return keypoints


if __name__ == "__main__":
    # Read frames from webcam
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
    cap.set(cv2.CAP_PROP_FPS, 30)

    # Create a window
    cv2.namedWindow("frame", cv2.WINDOW_NORMAL)

    # Show frames until 'q' is pressed
    while True:
        # Read frame
        ret, frame = cap.read()

        final_img = frame.copy()

        # Convert to HSV
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Show frame
        for color in ["red", "green", "blue", "yellow", "purple", "orange"]:
            thresh_img = threshold_for_color(hsv_img, color)
            keypoints = detect_blobs(thresh_img)

            nice_debug_img = cv2.max(frame, np.repeat(
                thresh_img[:, :, np.newaxis], 3, axis=2))
            for keypoint in keypoints:
                cv2.circle(final_img, (int(keypoint.pt[0]), int(keypoint.pt[1])),
                        int(keypoint.size/2), (0, 0, 0), 5)
        cv2.imshow("frame", final_img)

        # Check if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera
    cap.release()