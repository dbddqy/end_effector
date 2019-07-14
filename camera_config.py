import cv2
import numpy as np

left_camera_matrix = np.array([[437.97340, 0., 162.60558],
                               [0., 436.76128, 109.31669],
                               [0., 0., 1.]])
left_distortion = np.array([[-0.44538, 0.30825, 0.00348, -0.00043, 0.00000]])

right_camera_matrix = np.array([[443.31794, 0., 157.45938],
                                [0., 442.45644, 111.67415],
                                [0., 0., 1.]])

right_distortion = np.array([[-0.46754, 0.38003, 0.00028, -0.00104, 0.00000]])

om = np.array([0.00504, -0.02449, 0.00040])
R = cv2.Rodrigues(om)[0]
T = np.array([-60.89771, 0.42360, -0.94990])

size = (320, 240)


R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(left_camera_matrix, left_distortion,
                                                                  right_camera_matrix, right_distortion, size, R,
                                                                  T)

left_map1, left_map2 = cv2.initUndistortRectifyMap(left_camera_matrix, left_distortion, R1, P1, size, cv2.CV_16SC2)
right_map1, right_map2 = cv2.initUndistortRectifyMap(right_camera_matrix, right_distortion, R2, P2, size, cv2.CV_16SC2)
