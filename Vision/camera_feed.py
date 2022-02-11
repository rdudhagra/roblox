import argparse
import cv2

# Command line argument parsing
parser = argparse.ArgumentParser()
parser.add_argument("--cam_port", "-p", type=int, default=0, help="OpenCV camera port")
parser.add_argument("--cap_width", type=int, default=3840, help="Camera capture width")
parser.add_argument("--cap_height", type=int, default=2160, help="Camera capture height")
parser.add_argument("--cap_fps", type=int, default=30, help="Camera capture FPS")

if __name__ == "__main__":
    args = parser.parse_args()

    # Read frames from webcam
    cap = cv2.VideoCapture(args.cam_port)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.cap_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.cap_height)
    cap.set(cv2.CAP_PROP_FPS, args.cap_fps)

    # Show frames until 'q' is pressed
    frames_per_loop = 30 // args.cap_fps
    while True:
        for i in range(frames_per_loop):
            # Flush stale frames from framebuffer
            cap.grab()
        (ret, frame) = cap.retrieve()
        cv2.imshow("frame", frame)
        print("Frame captured: ", frame.shape)

        # Check if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera
    cap.release()
    cv2.destroyAllWindows()
