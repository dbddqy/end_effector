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


def get_3d_position(position_l_img, position_r_img):
    position_l_img_h = np.array([position_l_img[0], position_l_img[1], 1.])
    position_l_B = np.dot(R, np.dot(np.linalg.inv(left_camera_matrix), position_l_img_h)) + T
    position_l_A = T

    position_r_img_h = np.array([position_r_img[0], position_r_img[1], 1.])
    position_r_B = np.dot(np.linalg.inv(right_camera_matrix), position_r_img_h)
    position_r_A = np.zeros([3])

    intersect_r = get_intersect(position_l_A, position_l_B, position_r_A, position_r_B)
    intersect_l = get_intersect(position_r_A, position_r_B, position_l_A, position_l_B)

    return 0.5*intersect_r+0.5*intersect_l


def get_intersect(A, B, C, D):
    xA, yA, zA = A
    xB, yB, zB = B
    xC, yC, zC = C
    xD, yD, zD = D
    H = xB - xA
    I = yB - yA
    J = zB - zA
    K = xD - xC
    L = yD - yC
    M = zD - zC
    N = H*I*L-I*I*K-J*J*K+H*J*M
    O = H*H*L-H*L*K-I*J*M+J*J*L
    P = H*J*K-H*H*M-I*I*M+I*J*L
    Q = -xA*N+yA*O-zA*P
    k = (O*yC-N*xC-P*zC-Q)/(N*K-O*L+P*M)
    return np.array([K*k+xC, L*k+yC, M*k+zC])
