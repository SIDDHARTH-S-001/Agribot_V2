import numpy as np
import cv2
from openpyxl import Workbook  # Used for writing data into an Excel file
from sklearn.preprocessing import normalize

class StereoVision:
    def __init__(self, left_calibration_file, right_calibration_file):
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.criteria_stereo = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.kernel = np.ones((3, 3), np.uint8)
        self.objp = np.zeros((7 * 3, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:7, 0:3].T.reshape(-1, 2)
        self.objpoints = []  # 3d points in real world space
        self.imgpointsR = []  # 2d points in image plane
        self.imgpointsL = []
        self.wb = Workbook()
        self.ws = self.wb.active
        self.stereo = None
        self.stereoR = None
        self.wls_filter = None
        self.Left_Stereo_Map = None
        self.Right_Stereo_Map = None
        self.left_calibration_file = left_calibration_file
        self.right_calibration_file = right_calibration_file

    def load_calibration_matrices(self):
        # Load calibration matrices from the .txt files
        self.mtxL, self.distL = np.loadtxt(self.left_calibration_file, delimiter=',')
        self.mtxR, self.distR = np.loadtxt(self.right_calibration_file, delimiter=',')

    def calibrate_stereo(self):
        print('Starting stereo calibration... ')
        retS, _, _, _, _, _, _, _, _ = cv2.stereoCalibrate(
            self.objpoints, self.imgpointsL, self.imgpointsR, self.mtxL, self.distL, self.mtxR, self.distR, 
            ChessImaR.shape[::-1], criteria=self.criteria_stereo, flags=cv2.CALIB_FIX_INTRINSIC)
        RL, RR, PL, PR, _, _, _ = cv2.stereoRectify(self.mtxL, self.distL, self.mtxR, self.distR, 
                                                    ChessImaR.shape[::-1], _, _, _, rectify_scale=(0, 0))
        self.Left_Stereo_Map = cv2.initUndistortRectifyMap(self.mtxL, self.distL, RL, PL, 
                                                           ChessImaR.shape[::-1], cv2.CV_16SC2)
        self.Right_Stereo_Map = cv2.initUndistortRectifyMap(self.mtxR, self.distR, RR, PR, 
                                                            ChessImaR.shape[::-1], cv2.CV_16SC2)
        print('Stereo calibration completed.')

    def create_stereo_matcher(self):
        min_disp = 2
        num_disp = 130 - min_disp
        window_size = 3
        self.stereo = cv2.StereoSGBM_create(minDisparity=min_disp, numDisparities=num_disp, blockSize=window_size,
                                            uniquenessRatio=10, speckleWindowSize=100, speckleRange=32,
                                            disp12MaxDiff=5, P1=8 * 3 * window_size ** 2, P2=32 * 3 * window_size ** 2)
        self.stereoR = cv2.ximgproc.createRightMatcher(self.stereo)
        self.wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=self.stereo)
        self.wls_filter.setLambda(80000)
        self.wls_filter.setSigmaColor(1.8)

    def start_stereo_vision(self):
        CamR = cv2.VideoCapture(0)
        CamL = cv2.VideoCapture(2)
        while True:
            retR, frameR = CamR.read()
            retL, frameL = CamL.read()
            Left_nice = cv2.remap(frameL, self.Left_Stereo_Map[0], self.Left_Stereo_Map[1],
                                   interpolation=cv2.INTER_LANCZOS4, borderMode=cv2.BORDER_CONSTANT)
            Right_nice = cv2.remap(frameR, self.Right_Stereo_Map[0], self.Right_Stereo_Map[1],
                                    interpolation=cv2.INTER_LANCZOS4, borderMode=cv2.BORDER_CONSTANT)
            grayR = cv2.cvtColor(Right_nice, cv2.COLOR_BGR2GRAY)
            grayL = cv2.cvtColor(Left_nice, cv2.COLOR_BGR2GRAY)
            disp = self.stereo.compute(grayL, grayR)
            dispL = disp
            dispR = self.stereoR.compute(grayR, grayL)
            dispL = np.int16(dispL)
            dispR = np.int16(dispR)
            filteredImg = self.wls_filter.filter(dispL, grayL, None, dispR)
            filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX)
            filteredImg = np.uint8(filteredImg)
            disp = ((disp.astype(np.float32) / 16) - min_disp) / num_disp
            closing = cv2.morphologyEx(disp, cv2.MORPH_CLOSE, self.kernel)
            dispc = (closing - closing.min()) * 255
            dispC = dispc.astype(np.uint8)
            filt_Color = cv2.applyColorMap(filteredImg, cv2.COLORMAP_OCEAN)
            cv2.imshow('Filtered Color Depth', filt_Color)
            cv2.setMouseCallback("Filtered Color Depth", self.coords_mouse_disp, filt_Color)
            if cv2.waitKey(1) & 0xFF == ord(' '):
                break
        CamR.release()
        CamL.release()
        cv2.destroyAllWindows()

    def coords_mouse_disp(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDBLCLK:
            average = 0
            for u in range(-1, 2):
                for v in range(-1, 2):
                    average += disp[y + u, x + v]
            average /= 9
            Distance = -593.97 * average ** 3 + 1506.8 * average ** 2 - 1373.1 * average + 522.06
            Distance = np.around(Distance * 0.01, decimals=2)
            print('Distance: ' + str(Distance) + ' m')

# Instantiate the StereoVision class with the paths to the calibration files
left_calibration_file = "left_calibration.txt"
right_calibration_file = "right_calibration.txt"
stereo_vision = StereoVision(left_calibration_file, right_calibration_file)
stereo_vision.load_calibration_matrices()
stereo_vision.calibrate_stereo()
stereo_vision.create_stereo_matcher()
stereo_vision.start_stereo_vision()
