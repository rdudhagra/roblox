import cv2
import argparse
import math
import numpy as np
import pickle
import random
import tarfile
import time
from functools import reduce
from io import BytesIO

from video_capture_threading import VideoCaptureThreading

class ChessboardInfo:
    def __init__(self, n_cols = 0, n_rows = 0, dim = 0.0):
        self.n_cols = n_cols
        self.n_rows = n_rows
        self.dim = dim

def _pdist(p1, p2):
    """Distance between two points, p1 = (x, y), p2 = (x, y)"""
    return math.sqrt(math.pow(p1[0] - p2[0], 2) + math.pow(p1[1] - p2[1], 2))

def lmin(seq1, seq2):
    """Pairwise minimum of two sequences"""
    return [min(a, b) for (a, b) in zip(seq1, seq2)]

def lmax(seq1, seq2):
    """Pairwise maximum of two sequences"""
    return [max(a, b) for (a, b) in zip(seq1, seq2)]

def mean(seq):
    return sum(seq) / len(seq)

def _get_total_num_pts(boards):
    rv = 0
    for b in boards:
        rv += b.n_cols * b.n_rows
    return rv

def _get_outside_corners(corners, board):
    """Return the four corners of the board as a whole, as (up_left, up_right, down_right, down_left)"""
    xdim = board.n_cols
    ydim = board.n_rows

    if len(corners) != xdim * ydim:
        raise Exception("Invalid number of corners! %d corners, X: %d, Y: %d" %
                (len(corners), xdim, ydim))

    up_left = np.array(corners[0])
    up_right = np.array(corners[xdim - 1])
    down_right = np.array(corners[-1])
    down_left = np.array(corners[-xdim])

    return (up_left, up_right, down_right, down_left)

def _get_skew(corners, board):
    """Get skew for given checkerboard detection.
    Scaled to [0, 1], where 0 = no skew, 1 = high skew
    Skew is proportional to the divergence of three outside corners from 90 degrees.
    """
    (up_left, up_right, down_right, _) = _get_outside_corners(corners, board)

    def angle(a, b, c):
        """Return angle between lines ab, bc"""
        ab = a - b
        cb = c - b
        return math.acos(np.dot(ab, cb) / (np.linalg.norm(ab) * np.linalg.norm(cb)))

    skew = min(1.0, 2.0 * abs((math.pi / 2.0) - angle(up_left, up_right, down_right)))
    return skew



def _get_area(corners, board):
    """Get 2D image area of the detected checkerboarfd.
    The projected checckerboard is assumed to be a convex quadrilateral, and the area computed as
    |p X q| / 2
    """
    (up_left, up_right, down_right, down_left) = _get_outside_corners(corners, board)
    a = up_right - up_left
    b = down_right - up_right
    c = down_left - down_right
    p = b + c
    q = a + b
    return abs(p[0] * q[1] - p[1] * q[0]) / 2

def _get_corners(img, board, refine = True):
    """Get corners for a particular chessboard for an image"""
    (h, w) = img.shape[:2]
    mono = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    (ok, corners) = cv2.findChessboardCorners(mono, (board.n_cols, board.n_rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
    if corners is not None:
        corners = np.array([corner[0] for corner in corners], dtype=np.float32)

    # If any corners are within border pixels of the screen edge, reject the detection by setting ok to false
    # Note: This may cause issues with very low-res cameras, where 8 pixels is a non-negligible fraction
    # of the image size
    BORDER = 8
    if ok and not all([(BORDER < x < (w - BORDER)) and (BORDER < y < (h - BORDER)) for (x, y) in corners]):
        ok = False
    
    if refine and ok:
        # Use a radius of half the minimum distance between corners. This should be large enough to snap to the
        # correct corner, but not so large as to include a wrong corner in the search window
        min_distance = float("inf")
        for row in range(board.n_rows):
            for col in range(board.n_cols - 1):
                index = row * board.n_rows + col
                min_distance = min(min_distance, _pdist(corners[index], corners[index + 1]))
        for row in range(board.n_rows - 1):
            for col in range(board.n_cols):
                index = row * board.n_rows + col
                min_distance = min(min_distance, _pdist(corners[index], corners[index + board.n_cols]))
        radius = int(math.ceil(min_distance * 0.5))
        corners = cv2.cornerSubPix(mono, corners, (radius, radius), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1))

    return (ok, corners)

class Calibrator:
    """Base class for calibration system"""

    def __init__(self, boards, flags=0):
        # Make sure n_cols > n_rows to agree with OpenCV checkerboard detector output
        self._boards = [ChessboardInfo(max(i.n_cols, i.n_rows), min(i.n_cols, i.n_rows), i.dim) for i in boards]
        # Set to true after we perform calibration
        self.calibrated = False
        self.calib_flags = flags

        # self.db is list of (parameters, image) samples for use in calibration. Parameters has form
        # (X, Y, size, skew) all normalized to [0, 1], to keep track of what sort of samples we've taken
        # and ensure enough variety.
        self.db = []
        # For each db sample, we also record the detected corners.
        self.good_corners = []
        # Set to true when we have sufficiently varied samples to calibrate
        self.goodenough = False
        self.param_ranges = [0.7, 0.7, 0.4, 0.5]

    def mkgray(self, frame):
        """Convert a message into a bgr8 monochrome image.
        Deal with bayer images by converting to color, then to monochrome.
        """
        (cols, rows) = frame.shape[:2]
        mono = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        rgb = cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
        return rgb

    def get_parameters(self, corners, board, size):
        """Return list of parameters [X, Y, size, skew] describing the checkerboard view"""
        (width, height) = size
        # Compute some parameters for this chessboard
        Xs = [x for (x, y) in corners]
        Ys = [y for (x, y) in corners]
        area = _get_area(corners, board)
        border = math.sqrt(area)
        # For X and Y, we "shrink" the image all around by approx. half the board size
        # Otherwise large board are penalized because you can't get much X/Y variation
        p_x = min(1.0, max(0.0, (mean(Xs) - border / 2) / (width - border)))
        p_y = min(1.0, max(0.0, (mean(Ys) - border / 2) / (height - border)))
        p_size = math.sqrt(area / (width * height))
        skew = _get_skew(corners, board)
        params = [p_x, p_y, p_size, skew]
        return params

    def is_good_sample(self, params):
        """Returns True if the checkerboard detection detected by params should be added to the database."""
        if not self.db:
            return True

        def param_distance(p1, p2):
            return sum([abs(a - b) for (a, b) in zip(p1, p2)])

        db_params = [sample[0] for sample in self.db]
        d = min([param_distance(params, p) for p in db_params])
        return d > 0.2

    _param_names = ["X", "Y", "Size", "Skew"]

    def compute_goodenough(self):
        if not self.db:
            return None

        # Find range of checkerboard poses covered by samples in database
        all_params = [sample[0] for sample in self.db]
        min_params = reduce(lmin, all_params)
        max_params = reduce(lmax, all_params)
        # Don't reward small size or skew
        min_params = [min_params[0], min_params[1], 0.0, 0.0]

        # For each parameter, judge how much progress has been made towards adequate variation
        progress = [min((hi - lo) / r, 1.0) for (lo, hi, r) in zip(min_params, max_params, self.param_ranges)]
        # If we have lots of samples, allow calibration even if not all parameters are green
        self.goodenough = (len(self.db) >= 40) or all([p == 1.0 for p in progress])

        return zip(self._param_names, min_params, max_params, progress)

    def mk_object_points(self, boards, use_board_size = False):
        opts = np.zeros((_get_total_num_pts(boards), 3), dtype=np.float32)
        idx = 0
        for (i, b) in enumerate(boards):
            num_pts = b.n_cols * b.n_rows
            for j in range(num_pts):
                if use_board_size:
                    opts[idx + j, 0] = (j / b.n_cols) * b.dim
                    opts[idx + j, 1] = (j % b.n_cols) * b.dim
                    opts[idx + j, 2] = 0
                else:
                    opts[idx + j, 0] = (j / b.n_cols) * 1.0
                    opts[idx + j, 1] = (j % b.n_cols) * 1.0
                    opts[idx + j, 2] = 0
            idx += num_pts
        return opts

    def mk_image_points(self, good):
        total_pts = _get_total_num_pts([b for (_, b) in good])
        ipts = np.zeros((total_pts, 2), dtype=np.float32)

        idx = 0
        for (corners, _) in good:
            for j in range(len(corners)):
                ipts[idx + j, 0] = corners[j][0]
                ipts[idx + j, 1] = corners[j][1]
            idx += len(corners)

        return ipts

    def mk_point_counts(self, boards):
        npts = np.zeros((len(boards), 1), np.int32)
        for (i, board) in enumerate(boards):
            npts[i, 0] = board.n_cols * board.n_rows
        return npts

    def get_corners(self, img, refine = True):
        """Use cvFindChessboardCorners to find corners of chessboard in image.
        Check all boards. Return corners for first chessboard that it detects
        if given multiple size chessboards.

        Returns (ok, corners, board)
        """
        for b in self._boards:
            (ok, corners) = _get_corners(img, b, refine)
            if ok:
                return (ok, corners, b)
        return (False, None, None)

    def downsample_and_detect(self, rgb):
        """Downsample the input image to approximately VGA resolution and detect the
        calibration target corners in the full-size image.

        Combines these apparently orthogonal duties as an optimization. Checkerboard
        detection is too expensive on large images, so it's better to do detection on
        the smaller display image and scale the corners back up to the correct size.

        Returns (scrib, corners, downsampled_corners, board, (x_scale, y_scale))
        """
        # Scale the input image down to ~VGA size
        (height, width) = rgb.shape[:2]
        scale = math.sqrt((width * height) / (640.0 * 480.0))
        if scale > 1.0:
            scrib = cv2.resize(rgb, (int(width / scale), int(height / scale)))
        else:
            scrib = np.array(rgb)

        # Due to rounding, actual horizontal/vertical scaling may differ slightly
        x_scale = float(width) / scrib.shape[1]
        y_scale = float(height) / scrib.shape[0]

        # Detect checkerboard
        (ok, downsampled_corners, board) = self.get_corners(scrib, refine = True)

        # Scale corners back to full size image
        corners = None
        if ok:
            if scale > 1.0:
                # Refine up-scaled corners in the original full-res image
                corners_unrefined = [(c[0] * x_scale, c[1] * y_scale) for c in downsampled_corners]
                corners_unrefined = np.array(corners_unrefined, dtype=np.float32)
                mono = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
                radius = int(math.ceil(scale))
                corners = cv2.cornerSubPix(mono, corners_unrefined, (radius, radius), (-1, -1),
                        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1))
            else:
                corners = downsampled_corners

        return (scrib, corners, downsampled_corners, board, (x_scale, y_scale))
    

class MonoCalibrator(Calibrator):
    """Calibration class for monocular cameras"""

    def cal(self, images):
        """Calibrate camera from given images"""
        goodcorners = self.collect_corners(images)
        self.cal_fromcorners(goodcorners)

    def collect_corners(self, images):
        """Find chessboards in all images
        
        Arguments
            images [list of cvMat]: Source images containing chessboards

        Returns: (corners, ChessboardInfo)
        """
        self.size = images[0].shape[:2]
        corners = [self.get_corners[i] for i in images]

        goodcorners = [(co, b) for (ok, co, b) in corners if ok]
        if not goodcorners:
            raise RuntimeError("No corners found in images!")
        return goodcorners

    def cal_fromcorners(self, good):
        """Calibrate camera from corners

        Arguments
            good [(corners, ChessboardInfo)]: Good corner positions and boards
        """
        boards = [b for (_, b) in good]

        ipts = self.mk_image_points(good)
        opts = self.mk_object_points(boards)
        npts = self.mk_point_counts(boards)

        intrinsics = np.zeros((3, 3), np.float64)
        if self.calib_flags & cv2.CALIB_RATIONAL_MODEL:
            distortion = np.zeros((8, 1), np.float64) # rational polynomial
        else:
            distortion = np.zeros((5, 1), np.float64) # plumb bob

        intrinsics[0, 0] = 1.0
        intrinsics[1, 1] = 1.0
        cv2.calibrateCamera(np.expand_dims(opts, 0), np.expand_dims(ipts, 0), self.size, intrinsics, distortion,
            np.zeros((len(good), 3), dtype=np.float32),
            np.zeros((len(good), 3), dtype=np.float32),
            flags = self.calib_flags
        )

        self.intrinsics = intrinsics
        self.distortion = distortion

        # R is identity matrix for monocular calibration
        self.R = np.eye(3, dtype=np.float64)
        self.P = np.zeros((3, 4), dtype=np.float64)

        self.mapx = np.zeros(self.size, dtype=np.float32)
        self.mapy = np.zeros(self.size, dtype=np.float32)
        self.set_alpha(0.0)

    def set_alpha(self, a):
        """Set the alpha value for the calibrated camera solution. The alpha
        value is a zoom, and ranges from 0 (zoomed in, all pixels in calibrated
        image are valid) to 1 (zoomed out, all pixels in original image are in
        calibrated image).
        """
        ncm = self.P[:3,:3]
        # cv2.getOptimalNewCameraMatrix(self.intrinsics, self.distortion, self.size, a, ncm)
        cv2.initUndistortRectifyMap(self.intrinsics, self.distortion, self.R,
                ncm, self.size, cv2.CV_32FC1, self.mapx, self.mapy)

    def remap(self, src):
        """Apply the post-calibration undistortion to the source image

        Arguments
            src [cvMat]: Source image
        """
        r = np.array(src)
        cv2.remap(src, r, self.mapx, self.mapy)
        return r

    def undistort_points(self, src):
        """Apply the post-calibration undistortion to the source points

        Arguments
            src [cvMat]: N source pixel points (u,v) as an Nx2 matrix
        """
        dst = np.array(src)
        cv2.undistortPoints(src, dst, self.intrinsics, self.distortion, R = self.R, P = self.P)
        return dst

    def linear_error(self, corners, b):
        """Returns the linear error for a set of corners detected in the
        unrectified image
        """
        # P is checkerboard vertices as a list of (x,y) image points
        P = list(corners)

        def pt2line(x0, y0, x1, y1, x2, y2):
            """Point is (x0, y0), line is (x1, y1, x2, y2)"""
            return abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / \
                   math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        cc = b.n_cols
        cr = b.n_rows
        errors = []
        for r in range(cr):
            (x1, y1) = P[(cc * r) + 0]
            (x2, y2) = P[(cc * r) + cc - 1]
            for i in range(1, cc - 1):
                (x0, y0) = P[(cc * r) + i]
                errors.append(pt2line(x0, y0, x1, y1, x2, y2))
        return math.sqrt(sum([e**2 for e in errors]) / len(errors))


    def handle_frame(self, frame):
        """Detects the calibration target and, if found and provides enough
        new information, adds it to the sample database.

        Returns: The display image and progress info
        """
        rgb = self.mkgray(frame)
        linear_error = -1

        # Get display-image-to-be (scrib) and detection of the calibration target
        (scrib, corners, downsampled_corners, board, (x_scale, y_scale)) = \
                self.downsample_and_detect(rgb)

        if self.calibrated:
            # Show rectified image
            if x_scale != 1.0 or y_scale != 1.0:
                rgb_rect = self.remap(rgb)
                scrib = cv2.resize(rgb_rect, (scrib.shape[1], scrib.shape[0]))
            else:
                scrib = self.remap(rgb)

            if corners is not None and corners.shape[0] > 0:
                # Report linear error
                src = self.mk_image_points([(corners, board)])
                undistorted = list(cvmat_iterator(self.undistort_points(src)))
                linear_error = self.linear_error(undistorted, board)

                # Draw rectified corners
                scrib_src = [(x / x_scale, y / y_scale) for (x, y) in undistorted]
                cv2.drawChessboardCorners(scrib, (board.n_cols, board.n_rows), scrib_src, True)

        elif corners is not None and corners.shape[0] > 0:
            # Draw (potentially downsampled) corners onto display image
            src = self.mk_image_points([(downsampled_corners, board)])
            cv2.drawChessboardCorners(scrib, (board.n_cols, board.n_rows), src, True)

            # Add sample to database only if it's sufficiently different from any previous sample
            params = self.get_parameters(corners, board, rgb.shape[:2])
            if self.is_good_sample(params):
                self.db.append((params, rgb))
                self.good_corners.append((corners, board))
                print("***Added sample %d, p_x = %.3f, p_y = %.3f, p_size = %.3f, skew = %.3f" %
                        tuple([len(self.db)] + params))

        return (scrib, self.compute_goodenough(), linear_error)

    def do_calibration(self, dump = False):
        if not self.good_corners:
            print("**** Collecting corners for all images! ****")
            images = [i for (p, i) in self.db]
            self.good_corners = self.collect_corners(images)
        # Dump should only occur if the user wants it
        self.size = self.db[0][1].shape[:2]
        self.cal_fromcorners(self.good_corners)
        self.calibrated = True
        if dump:
            with open("camera_calibration_%08x.pkl" % random.getrandbits(32), "wb") as f:
                pickle.dump((self.size, self.good_corners), f)

    def do_tarfile_save(self, tf):
        """Write images and calibration solution to a tarfile object"""
        def taradd(name, buf):
            if type(buf) == str:
                buf = buf.encode("utf-8")
            s = BytesIO(buf)

            ti = tarfile.TarInfo(name)
            ti.size = len(buf)
            ti.uname = "calibrator"
            ti.mtime = int(time.time())
            tf.addfile(tarinfo = ti, fileobj = s)

        ims = [("left-%04d.png" % i, im) for (i, (_, im)) in enumerate(self.db)]
        for (name, im) in ims:
            taradd(name, cv2.imencode(".png", im)[1].tostring())
        
        taradd("ost.txt", self.ost())

    def do_tarfile_calibration(self, filename):
        archive = tarfile.open(filename, "r")
        limages = [image_from_archive(archive, f) for f in archive.getnames()
                   if (f.startswith("left ") and (f.endswith(".pgm") or f.endswith("png")))]
        self.cal(limages)

    def do_save(self):
        filename = "calibrationdata.tar.gz"
        tf = tarfile.open(filename, "w:gz")
        self.do_tarfile_save(tf)
        tf.close()
        print("Wrote calibration data to", filename)

    def ost(self):
        return self.lrost("left", self.distortion, self.intrinsics, self.R, self.P)

    def lrost(self, name, d, k, r, p):
        msg = f"""# oST version 5.0 parameters


        [image]
        width
        {self.size[0]}
        height
        {self.size[1]}

        [narrow_stereo/{name}]

        camera matrix
        {" ".join(["%8f" % k[0, i] for i in range(3)])}
        {" ".join(["%8f" % k[1, i] for i in range(3)])}
        {" ".join(["%8f" % k[2, i] for i in range(3)])}

        distortion
        {" ".join(["%8f" % d[i, 0] for i in range(len(d))])}

        rectification
        {" ".join(["%8f" % r[0, i] for i in range(3)])}
        {" ".join(["%8f" % r[1, i] for i in range(3)])}
        {" ".join(["%8f" % r[2, i] for i in range(3)])}

        projection
        {" ".join(["%8f" % p[0, i] for i in range(4)])}
        {" ".join(["%8f" % p[1, i] for i in range(4)])}
        {" ".join(["%8f" % p[2, i] for i in range(4)])}
        """
        return msg


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cam_port", "-p", type=int, default=0, help="OpenCV camera port")
    parser.add_argument("--cap_width", type=int, default=10000, help="Camera capture width")
    parser.add_argument("--cap_height", type=int, default=10000, help="Camera capture height")
    parser.add_argument("--cap_fps", type=int, default=30, help="Camera capture FPS")
    args = parser.parse_args()

    cap = VideoCaptureThreading(src=args.cam_port, width=args.cap_width, height=args.cap_height, fps=args.cap_fps).start()
    
    # Initialize monocular calibrator with 8x6 chessboard
    calibrator = MonoCalibrator([ChessboardInfo(8, 6, 0.108)])

    while True:
        # Read frame
        (ret, frame) = cap.read()

        # Feed frame into calibrator
        (scrib, params, linear_error) = calibrator.handle_frame(frame)
        cv2.imshow("scrib", scrib)

        # Check if calibration is good enough
        calib_result = calibrator.compute_goodenough()
        calib_msg = " / ".join(["%s: %.3f-%.3f [%.1f]" % elt for elt in calib_result])
        print(calib_msg)

        if calibrator.goodenough:
            print("Enough samples collected, performing calibration...")
            calibrator.do_calibration(dump=True)
            calibrator.do_save()
            break

        # Check if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Stop the camera
    cap.stop()
    cv2.destroyAllWindows()
