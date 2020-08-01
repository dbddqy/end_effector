# Importing the libraries
import cv2

# Loading the cascades
face_cascade = cv2.CascadeClassifier('./haarcascade_frontalface_default.xml')


# Defining a function that will do the detections
def detect_face(img):
    face_img = img.copy()

    faces = face_cascade.detectMultiScale(face_img, scaleFactor=1.2, minNeighbors=5)

    centers = []
    for (x, y, w, h) in faces:
        cv2.rectangle(face_img, (x, y), (x + w, y + h), (255, 0, 0), 3)
        centers.append([x+0.5*w, y+0.5*h])

    return (centers, face_img)
