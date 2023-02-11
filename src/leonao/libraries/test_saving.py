import cv2

img = cv2.imread("../watchfolder/sketch_face.jpg")
cv2.imwrite("../watchfolder/sketch_face.jpg", img)