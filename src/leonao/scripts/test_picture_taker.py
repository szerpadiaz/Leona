#!/usr/bin/env python
import rospy
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from naoqi import ALProxy
from naoqi_bridge_msgs.msg import HeadTouch

################### Variables ###################
USE_MEDIA_PIPE_DIRECT = False
WATCHFOLDER_PATH = "/home/hrsa/watchfolder/"

IMAGE_ROTATION = cv2.ROTATE_90_COUNTERCLOCKWISE
# cv2.ROTATE_90_COUNTERCLOCKWISE
# cv2.ROTATE_180
# cv2.ROTATE_90_CLOCKWISE

if USE_MEDIA_PIPE_DIRECT:
    from face_detector import FaceDetector

class pictureTaker:
    def __init__(self, local = False):
        self.local = local
        self.minFaceSize = 0.33
        self.minBrightness = 100
        self.maxBrightness = 200
        self.minContrast = 50
        self.front_button_pressed = False
        if local:
            self.camera = cv2.VideoCapture(0)
        if not self.local:
            # Importing only if neccessary to easier run locally
            import rospy
            from sensor_msgs.msg import Image
            from cv_bridge import CvBridge
            self.robot_ip=str(os.getenv("NAO_IP"))
            self.robot_port=int(9559)
            self.tts = ALProxy("ALTextToSpeech", self.robot_ip, 9559)
            self.bridge = CvBridge()
            self.image_sub = rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, self.newImageCallback)
            self.head_sub = rospy.Subscriber('/tactile_touch', HeadTouch, self.head_touch_callback)
            print("Picuture Taker initialized")

    def takePicture(self, path):
        if self.local:
            _, frame = self.camera.read()
            cv2.imwrite(path, frame)
            return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        else:
            img = self.bridge.imgmsg_to_cv2(self.currentImageFromStream, desired_encoding='bgr8')
            if IMAGE_ROTATION:
                img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
            with open(WATCHFOLDER_PATH + "result.txt", "w") as f: # Reset the observation results
                f.write("")
            cv2.imwrite(WATCHFOLDER_PATH+path, img)  
            print("Image saved in " + WATCHFOLDER_PATH + path)")
            return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    def analyzePicture(self, img, showAnalysis = False):
        bbox = ""
        while bbox == "":
            with open(WATCHFOLDER_PATH + "result.txt", "r") as f:
                bbox = f.read()
                # if no face is found, bbox is "None"
                if bbox == "None":
                    bbox = np.array(None)
                    break

                # If the analysis is not yet one, bbox is empty
                if bbox == "":
                    continue
                
                # Standardizing input
                bbox = bbox.replace("[  ", "")
                bbox = bbox.replace("[ ", "")
                bbox = bbox.replace("[", "")
                bbox = bbox.replace("]", "")
                bbox = bbox.replace("   ", " ")
                bbox = bbox.replace("  ", " ")
                bbox = bbox.split(" ")
                bbox = [int(i) for i in bbox]
                bbox = np.array(bbox)
                print("Face recognized at:", bbox)

        if (bbox == None).any():
            self.speak("I could not see your face, there is probably too little contrast.")
            return "Error: No face found in picture", None
        
        # Check if the bounding box in bbox is large enough
        # should be at least 1/3 of the picture
        imgHeight, imgWidth, _ = img.shape
        bboxMinSize = min(imgHeight, imgWidth) * self.minFaceSize
        if bbox[2] < bboxMinSize or bbox[3] < bboxMinSize:
            self.speak("Face too small, please come closer")
            return "Error: Face too small, please come closer", None

        # Check if the face is too dark
        face = img[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]]
        faceBrightness = np.mean(face)
        if faceBrightness < self.minBrightness:
            self.speak("Face too dark, try to get more light")
            return "Error: Face too dark, try to get more light", None

        # Check if the face is too bright
        if faceBrightness > self.maxBrightness:
            self.speak("Face too bright, try to get less light")
            return "Error: Face too bright, try to get less light", None

        # Check if the face has too little contrast
        faceContrast = np.std(face)
        if faceContrast < self.minContrast:
            self.speak("Face too little contrast, try to get more contrast")
            return "Error: Face too little contrast, try to get more contrast", None
        
        self.speak("Looks good!")
        return "Success", img

    def speak(self, text):
        print(text)
        if self.local:
            os.system(f'say "{text}"')
        else:
            self.tts.say(text)

    def main_loop(self):
        while not (self.front_button_pressed):
            rospy.sleep(0.2)
        self.speak("Taking a picture in 3, 2, 1. Smile!")
        img = self.takePicture("detect_face.jpg")
        analyzePictureResponse, img = self.analyzePicture(img, showAnalysis= True)
        if analyzePictureResponse == "Success":
            cv2.imwrite(WATCHFOLDER_PATH+"sketch_face.jpg", img) 
            pass
        else:
            self.speak("Let's try again!")

    ################ Running Callbacks ################

    def head_touch_callback(self, head_touch_event):
        self.front_button_pressed = head_touch_event.button == HeadTouch.buttonFront and head_touch_event.state == HeadTouch.statePressed

    def newImageCallback(self, img_msg):
        self.currentImageFromStream = img_msg 

if __name__ == '__main__':
    rospy.init_node('test_picture_taker', anonymous=False)
    pt = pictureTaker(local = False)
    try:
        while not rospy.is_shutdown():
            pt.main_loop()      
    except rospy.ROSInterruptException:
        pass