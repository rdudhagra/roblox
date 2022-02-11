import cv2
import pickle
import threading

#################################################################################################################
#################################################################################################################       
#### Source: https://github.com/gilbertfrancois/video-capture-async/blob/master/main/gfd/py/video/capture.py ####
#################################################################################################################
#################################################################################################################

class VideoCaptureThreading:
    def __init__(self, port=0, width=1920, height=1080, fps=30, calib=""):
        self.port = port
        self.cap = cv2.VideoCapture(self.port)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.grabbed, self.frame = self.cap.read()
        self.started = False
        self.read_lock = threading.Lock()

        if calib is not None and calib != "":
            with open(calib, "rb") as f:
                self.calib = pickle.load(f)

    def set(self, var1, var2):
        self.cap.set(var1, var2)

    def start(self):
        if self.started:
            print('[!] Threaded video capturing has already been started.')
            return None
        self.started = True
        self.thread = threading.Thread(target=self.update, args=())
        self.thread.start()
        return self

    def update(self):
        while self.started:
            grabbed, frame = self.cap.read()
            with self.read_lock:
                self.grabbed = grabbed
                self.frame = frame

    def read(self):
        with self.read_lock:
            frame = self.frame.copy()
            grabbed = self.grabbed
        return grabbed, frame

    def read_calib(self):
        grabbed, frame = self.read()
        frame = cv2.undistort(frame, self.calib["camera_matrix"], self.calib["distortion"])
        return grabbed, frame

    def stop(self):
        self.started = False
        self.thread.join()

    def __exit__(self, exec_type, exc_value, traceback):
        self.cap.release()
