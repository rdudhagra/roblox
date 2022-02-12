import apriltag
import argparse
import cv2
import numpy as np

from video_capture_threading import VideoCaptureThreading as VideoCapture

detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))

def detect_apriltags(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(gray)
    return tags

def apply_transform(transform, point):
    x = np.array([point[0], point[1], 1])
    out = transform @ x
    return out[:2]

def test_img2world_transform(tags, transform, w=1, h=1):
    # Tests the accuracy of the img2world transform

    # Find tag centers in image
    c_img = {}
    for tag in tags:
        if tag.tag_id in [0,1,2,3]:
            c_img[tag.tag_id] = tag.center

    # Abort if four corners of field are not visible
    for tag_id in [0,1,2,3]:
        if tag_id not in c_img:
            return None

    # Specify actual tag centers in world
    c_world = {0: (0, 0), 1: (w, 0), 2: (0, h), 3: (w, h)}

    # Find error between actual and expected corner positions
    error = 0
    for (tag_id, img_pos) in c_img.items():
        world_pos_exp = c_world[tag_id]
        world_pos_act = apply_transform(img2world, img_pos)
        error += np.linalg.norm(world_pos_exp - world_pos_act)
        print(f"Tag {tag_id}: Expected {tuple(world_pos_exp)}, Actual {tuple(world_pos_act)}")

    print(f"Error: {error / len(c_img)}")

def get_img2world_transform(tags, w=1, h=1):
    # Uses linear least squares to fit an image-to-world affine transform to the given tags
    # Assumes tag1 (0, 0), tag2 (w, 0), tag3 (0, h), tag4 (w, h)

    # Find tag centers in image
    c = {}
    for tag in tags:
        c[tag.tag_id] = tag.center

    # Abort if four corners of field are not visible
    for key in [0,1,2,3]:
        if key not in c:
            return None

    # Build homography matrices
    # Reference: http://cs.cmu.edu/~16385/lectures/lecture7.pdf, slide 79
    b = np.transpose(np.array([0, 0, w, 0, 0, h, w, h]))
    A = np.array([
        [c[0][0], c[0][1], 1, 0, 0, 0], # tag0_image
        [0, 0, 0, c[0][0], c[0][1], 1],
        [c[1][0], c[1][1], 1, 0, 0, 0], # tag1_image
        [0, 0, 0, c[1][0], c[1][1], 1],
        [c[2][0], c[2][1], 1, 0, 0, 0], # tag2_image
        [0, 0, 0, c[2][0], c[2][1], 1],
        [c[3][0], c[3][1], 1, 0, 0, 0], # tag3_image
        [0, 0, 0, c[3][0], c[3][1], 1],
    ])

    # Solve linear least squares
    AT = np.transpose(A)
    x = np.linalg.inv(AT @ A) @ AT @ b

    # Construct 3x3 matrix from solution
    img2world = np.array([
        [x[0], x[1], x[2]],
        [x[3], x[4], x[5]],
        [0, 0, 1],
    ])
    return img2world

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cam_port", "-p", type=int, default=0, help="OpenCV camera port")
    parser.add_argument("--cap_width", "-x", type=int, default=3840, help="Camera capture width")
    parser.add_argument("--cap_height", "-y", type=int, default=2160, help="Camera capture height")
    parser.add_argument("--cap_fps", "-f", type=int, default=30, help="Camera capture FPS")
    parser.add_argument("--cam_calib", "-c", type=str, default="camera_calibration_data.pkl", help="Camera calibration")
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
        ret, frame = cap.read()
        cv2.imshow("frame", frame)

        tags = detect_apriltags(frame)
        img2world = get_img2world_transform(tags)
        test_img2world_transform(tags, img2world)

        # Check if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera
    cap.stop()
    cv2.destroyAllWindows()
