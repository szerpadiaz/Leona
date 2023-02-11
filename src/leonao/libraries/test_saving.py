"""
A shrot program to simply test triggering the watchdog through a cv2,imwrite 
We do this, as we were having problems through double triggering of events
"""


import cv2

img = cv2.imread("../watchfolder/sketch_face.jpg")
cv2.imwrite("../watchfolder/sketch_face.jpg", img)