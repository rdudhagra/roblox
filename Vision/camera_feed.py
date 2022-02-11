import argparse
import cv2

from video_capture_threading import VideoCaptureThreading

if __name__ == "__main__":
    # Command line argument parsing
    parser = argparse.ArgumentParser()
    parser.add_argument("--cam_port", "-p", type=int, default=0, help="OpenCV camera port")
    parser.add_argument("--cap_width", type=int, default=3840, help="Camera capture width")
    parser.add_argument("--cap_height", type=int, default=2160, help="Camera capture height")
    parser.add_argument("--cap_fps", type=int, default=30, help="Camera capture FPS")
    args = parser.parse_args()

    # Read frames from webcam
    cap = VideoCaptureThreading(src=args.cam_port, width=args.cap_width, height=args.cap_height, fps=args.cap_fps).start()

    # Show frames until 'q' is pressed
    while True:
        # Read frame
        ret, frame = cap.read()
        cv2.imshow("frame", frame)
        print("Frame captured: ", frame.shape)

        # Check if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera
    cap.stop()
    cv2.destroyAllWindows()
