import numpy as np
import cv2
import camera_config as cc
import feature_detection as fd
import detect_color as dc

def vision_init():
    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    return camera

def detect_position_face(camera):
    ret, frame = camera.read()
    left_frame = frame[0:240, 0:320]
    right_frame = frame[0:240, 320:640]

    # Rectify photos to images
    img1_rectified = cv2.remap(left_frame, cc.left_map1, cc.left_map2, cv2.INTER_LINEAR)
    img2_rectified = cv2.remap(right_frame, cc.right_map1, cc.right_map2, cv2.INTER_LINEAR)

    # Convert to grey images
    imgL = cv2.cvtColor(img1_rectified, cv2.COLOR_BGR2GRAY)
    imgR = cv2.cvtColor(img2_rectified, cv2.COLOR_BGR2GRAY)

    # Get the center of face and the images that contains detected face(s)
    positions_imgL, imgL_face = fd.detect_face(imgL)
    positions_imgR, imgR_face = fd.detect_face(imgR)

    # If the left image and right image detect different amount of faces, we ignore the pair
    if len(positions_imgL) != len(positions_imgR):
        return ([], imgL_face, imgR_face)

    # Sort image locations based on their coordinate
    positions_imgL_sorted = sorted(positions_imgL, key=lambda x: x[0], reverse=False)
    positions_imgR_sorted = sorted(positions_imgR, key=lambda x: x[0], reverse=False)

    # If the location sorted is still not in the right order (which will return a ridiculous location), we ignore the pair
    for index in range(len(positions_imgR)):
        position = cc.get_3d_position(positions_imgL_sorted[index], positions_imgR_sorted[index])
        if not 800 > position[2] > 100:
            return ([], imgL_face, imgR_face)

    # Otherwise, we return the locations detected
    positions = []

    for index in range(len(positions_imgR)):
        position = cc.get_3d_position(positions_imgL_sorted[index], positions_imgR_sorted[index])
        positions.append(position)

    return (positions, imgL_face, imgR_face)


def detect_position_face_multiple_times(camera, number_of_times, index_image):
    result_lst = []
    imgL_lst = []
    imgR_lst = []
    for _ in range(number_of_times):
        result, imgL, imgR = detect_position_face(camera)
        if len(result) > 0:
            result_lst.append(result)
            imgL_lst.append(imgL)
            imgR_lst.append(imgR)
    if len(result_lst) == 0:
        print("empty")
        return []

    # Get the maximum number of faces detected from each image in all images
    max_faces = max([len(elem) for elem in result_lst])
    print("max_face: " + str(max_faces))

    # Filter the results to get only the result of maximum faces
    # Disgard all the images that fail to detect all faces
    filtered_result = []
    filtered_imgL_lst = []
    filtered_imgR_lst = []
    for i in range(len(result_lst)):
        if len(result_lst[i]) == max_faces:
            filtered_result.append(result_lst[i])
            filtered_imgL_lst.append(imgL_lst[i])
            filtered_imgR_lst.append(imgR_lst[i])

    # Get median for each coordinates
    final_result = []
    pictures = []
    for pic in filtered_result:
        pictures.append(Picture(pic))
    for j in range(len(pictures[0].positions)):
        x_temp = []
        y_temp = []
        z_temp = []
        for i in range(len(pictures)):
            x_temp.append(pictures[i].positions[j].x)
            y_temp.append(pictures[i].positions[j].y)
            z_temp.append(pictures[i].positions[j].z)
        final_result.append([median(x_temp), median(y_temp), median(z_temp)])

    # Save images
    cv2.imwrite("/home/pi/Desktop/imgL_%d.jpg" % index_image, filtered_imgL_lst[0])
    cv2.imwrite("/home/pi/Desktop/imgR_%d.jpg" % index_image, filtered_imgR_lst[0])

    return final_result


# Helper function to calculate median
def median(lst):
    n = len(lst)
    s = sorted(lst)
    return (sum(s[n//2-1:n//2+1])/2.0, s[n//2])[n % 2] if n else None

# Helper classes
class Position:
    def __init__(self, position):
        self.x = position[0]
        self.y = position[1]
        self.z = position[2]

# Helper classes
class Picture:
    def __init__(self, positions):
        self.positions = []
        for position in positions:
            self.positions.append(Position(position))

# Helper function to close the camera
def camera_release(camera):
    camera.release()
    cv2.destroyAllWindows()
