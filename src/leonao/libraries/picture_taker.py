import cv2
import numpy as np
import os
import time

import cv2
import matplotlib.pyplot as plt
from face_detector import FaceDetector

# The picture taker module is responsible for taking pictures
## The module should provide for the following tasks as seperate functions:
## Tell the model that the picture will be taken
## Take a picture and save it
## Analse the picture if it is of high enough quality and return the result (how the picture can be improved)
### Try to find a face in the picture

class pictureTaker:
    def __init__(self, camera, local:bool = False):
        self.local = local
        self.camera = camera
        self.minFaceSize = 0.33
        self.minBrightness = 100
        self.maxBrightness = 200
        self.minContrast = 50
        # Might have to change later
        if not self.local:
            import rospy
            from sensor_msgs.msg import Image
            from cv_bridge import CvBridge
            self.bridge = CvBridge()
            self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)

    def takePicture(self, path, img_msg = None):
        if self.local:
            ret, frame = self.camera.read()
            # delete the content of the result.txt file
            with open("src/leonao/libraries/watchfolder/result.txt", "w") as f:
                f.write("")
            
            cv2.imwrite(path, frame)
            return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        else:
            # Convert raw image data to cv mat BGR8
             img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
    
    def analyzePicture(self, img, showAnalysis:bool = False):
        # Show the picture with pyplot if showAnalysis is True
        if showAnalysis:
            plt.imshow(img)
            plt.xticks([]), plt.yticks([])
            plt.draw()

        # Analyze the picture and return the result
        # try to find a face in the picture
        


        # Read the result.txt file in watchfolder
        # The result.txt file contains the result of the face detection
        bbox = ""
        while bbox == "":
            with open("src/leonao/libraries/watchfolder/result.txt", "r") as f:
                bbox = f.read()
                if bbox == "":
                    continue
                bbox = bbox.replace("[", "")
                bbox = bbox.replace("]", "")
                bbox = bbox.split(" ")
                bbox = [int(i) for i in bbox]
                bbox = np.array(bbox)
                print(bbox)

        if (bbox == None).any():
            print("No face found in picture")
            return "Error: No face found in picture", None
        
        # Check if the bounding box in bbox is large enough
        # should be at least 1/3 of the picture

        # Get the size of the picture
        imgHeight, imgWidth, imgChannels = img.shape
        # Get the size of the bounding box
        bboxMinSize = min(imgHeight, imgWidth) * self.minFaceSize
        print(f'Face is {bbox[2]}x{bbox[3]} pixels, thats {bbox[2]/imgWidth*100}% of the width and {bbox[3]/imgHeight*100}% of the height')

        if bbox[2] < bboxMinSize or bbox[3] < bboxMinSize:
            print("Face too small, please come closer")
            self.speak("Face too small, please come closer")
            return "Error: Face too small, please come closer", None

        # Check if the face is too dark
        # Get the average brightness of the face   
        face = img[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]]
        faceBrightness = np.mean(face)
        print(f'Face brightness is {faceBrightness}')
        if faceBrightness < self.minBrightness:
            self.speak("Face too dark, try to get more light")
            return "Error: Face too dark, try to get more light", None

        # Check if the face is too bright
        if faceBrightness > self.maxBrightness:
            self.speak("Face too bright, try to get less light")
            return "Error: Face too bright, try to get less light", None

        # Check if the face has too little contrast
        # Get the standard deviation of the face
        faceContrast = np.std(face)
        print(f'Face contrast is {faceContrast}')
        if faceContrast < self.minContrast:
            self.speak("Face too little contrast, try to get more contrast")
            return "Error: Face too little contrast, try to get more contrast", None
        return "Success", img

    def speak(self, text):
        os.system(f'say "{text}"')


# Example usage
if __name__ == "__main__":
    # Take a picture with the pictureTaker
    pt = pictureTaker(cv2.VideoCapture(0), local=True)
    img = pt.takePicture("src/leonao/libraries/watchfolder/tmp_picture.jpg")
    analyzePicture = pt.analyzePicture(img, showAnalysis= True)