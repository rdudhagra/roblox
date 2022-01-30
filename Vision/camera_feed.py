import cv2
import argparse


parser = argparse.ArgumentParser()
parser.add_argument("--cam_port", "-p", type=int, default=0, help="OpenCV camera port")
parser.add_argument("--cap_width", type=int, default=10000, help="Camera capture width")
parser.add_argument("--cap_height", type=int, default=10000, help="Camera capture height")
parser.add_argument("--cap_fps", type=int, default=15, help="Camera capture FPS")
args = parser.parse_args()

cam = cv2.VideoCapture(args.cam_port)

# Set camera properties
cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'));
cam.set(cv2.CAP_PROP_FRAME_WIDTH, args.cap_width)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, args.cap_height)
cam.set(cv2.CAP_PROP_FPS, args.cap_fps)

while True:
    frames_per_loop = 30 // args.cap_fps
    for i in range(frames_per_loop):
        cam.grab()
    (ret, frame) = cam.retrieve()
    cv2.imshow("frame", frame)
    print("Frame captured: ", frame.shape)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam.release()
cv2.destroyAllWindows()
