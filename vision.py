import numpy as np
import cv2
import camera_config as cc
import detect_color as dc


def vision_init():
    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    return camera


def detect_position_green(camera):
    ret, frame = camera.read()
    left_frame = frame[0:240, 0:320]
    right_frame = frame[0:240, 320:640]

    img1_rectified = cv2.remap(left_frame, cc.left_map1, cc.left_map2, cv2.INTER_LINEAR)
    img2_rectified = cv2.remap(right_frame, cc.right_map1, cc.right_map2, cv2.INTER_LINEAR)

    imgL = cv2.cvtColor(img1_rectified, cv2.COLOR_BGR2GRAY)
    imgR = cv2.cvtColor(img2_rectified, cv2.COLOR_BGR2GRAY)
    # cv2.imshow("L", imgL)

    stereo = cv2.StereoSGBM_create(0,     # minDisparity
                                   16*6,  # numDisparities
                                   5,     # blockSize
                                   8*1*25,   # P1
                                   32*1*25,  # P2
                                   1,    # disp12MaxDiff
                                   63,    # preFilterCqap
                                   1,     # uniquenessRatio
                                   50,   # speckleWindowSize
                                   2,    # speckleRange
                                   True)     # mode
    disparity = stereo.compute(imgL, imgR)

    # disp = cv2.normalize(disparity, disparity, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    # cv2.imshow("disp", disp)

    threeD = cv2.reprojectImageTo3D(disparity.astype(np.float32) / 16., cc.Q)

    green = dc.detect_green(img1_rectified)
    # cv2.imshow("green", green)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    binary = cv2.morphologyEx(green, cv2.MORPH_OPEN, kernel)
    # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
    # binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    _, cnts, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if(len(cnts) == 1):
        c = cnts[0]
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return threeD[cY][cX]
    else:
        return [0, 0, 0]


def detect_position_mouth():
    pass


def camera_release(camera):
    camera.release()
    cv2.destroyAllWindows()
