import numpy as np
import cv2
import os

# Define the number of corners in the calibration pattern (e.g., a chessboard)
pattern_size = (7, 3)  # Number of inner corners in x and y directions

# Create arrays to store object points and image points from all images
obj_points = []  # 3D points in real world space
img_points = []  # 2D points in image plane
images_folder = "images"

# Prepare object points, like (0,0,0), (1,0,0), (2,0,0), ..., (6,5,0)
objp = np.zeros((np.prod(pattern_size), 3), dtype=np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)

# Load saved images from the folder
image_files = os.listdir(images_folder)

# Iterate through saved images
for image_file in image_files:
    image_path = os.path.join(images_folder, image_file)
    frame = cv2.imread(image_path)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Find chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

    # If found, add object points, image points (after refining them)
    if ret:
        obj_points.append(objp)
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                           criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        img_points.append(corners_refined)

# Perform camera calibration
if len(obj_points) > 0:
    # Capture dimensions of one of the images
    img_shape = gray.shape[::-1]

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, img_shape, None, None)

    # Print the calibration results
    print("Camera matrix:")
    print(mtx)
    print("\nDistortion coefficients:")
    print(dist)

    # Save camera matrix and distortion coefficients to a text file
    np.savetxt("camera_matrix.txt", mtx)
    np.savetxt("distortion_coefficients.txt", dist)
else:
    print("No images to calibrate.")
