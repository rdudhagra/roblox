import argparse
import cv2
import numpy as np

from detect_apriltags import detect_apriltags, get_img2world_transform, get_robot_poses
from detect_cubes import preprocess_frame, threshold_for_color, detect_squares, get_cube_poses, draw_squares
from video_capture_threading import VideoCaptureThreading as VideoCapture

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

        # Detect apriltags and find img2world transform for cube height corners
        tags = detect_apriltags(frame)

        # Find img2world transforms
        img2world_cube = get_img2world_transform(tags, "cube")
        img2world_robot = get_img2world_transform(tags, "robot")

        # Find robot poses
        get_robot_poses(tags, img2world_robot)

        # Detect cubes
        hsv_img = preprocess_frame(frame)

        all_thresh = None
        all_cubes = {}
        for color in ["red", "green", "blue", "yellow", "purple", "orange"]:
            thresh_img = threshold_for_color(hsv_img, color)
            all_thresh = thresh_img if all_thresh is None else np.bitwise_or(all_thresh, thresh_img)

            squares = detect_squares(thresh_img)
            cubes = get_cube_poses(squares, img2world_cube)
            all_cubes[color] = cubes

            draw_squares(frame, squares)

        cv2.imshow("frame", frame)

        for (color, cubes) in all_cubes.items():
            print(f"{color} cubes: {' / '.join([repr(((x, y), th * 180 / np.pi)) for ((x, y), th) in cubes])}")

        # Check if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera
    cap.stop()
    cv2.destroyAllWindows()
