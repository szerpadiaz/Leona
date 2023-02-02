#!/usr/bin/env python
import rospy
import os
import cv2
import numpy as np
from naoqi import ALProxy
import pickle

################### Variables ###################
USE_MEDIA_PIPE_DIRECT = False

WATCHFOLDER_PATH = "/home/hrsa/leonao/src/leonao/watchfolder/"
SKETCH_FACE_FILE = WATCHFOLDER_PATH + "sketch_face.jpg"
SKETCH_FACE_PATHS_FILE = WATCHFOLDER_PATH + "sketcher_result.pkl"


if USE_MEDIA_PIPE_DIRECT:
    from face_detector import FaceDetector

class pictureTaker:
    def __init__(self, local = False, useTestPicture = False):
        self.local = local
        self.useTestPicture = useTestPicture
        if useTestPicture:
            self.IMAGE_ROTATION = False
        else:
            self.IMAGE_ROTATION = cv2.ROTATE_90_COUNTERCLOCKWISE
            # cv2.ROTATE_90_COUNTERCLOCKWISE
            # cv2.ROTATE_180
            # cv2.ROTATE_90_CLOCKWISE
        self.minFaceSize = 0.33
        self.minBrightness = 100
        self.maxBrightness = 200 
        self.minContrast = 70
        if local:
            self.camera = cv2.VideoCapture(0)
        if not self.local:
            # Importing only if neccessary to easier run locally
            import rospy
            from sensor_msgs.msg import Image
            from cv_bridge import CvBridge
            self.bridge = CvBridge()
            self.robot_ip=str(os.getenv("NAO_IP"))
            self.robot_port=int(9559)
            self.tts = ALProxy("ALTextToSpeech", self.robot_ip, 9559)
            self.image_sub = rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, self.newImageCallback)
            print("Picuture Taker initialized")

    def takePicture(self, path):
        if self.local:
            _, frame = self.camera.read()
            cv2.imwrite(path, frame)
            return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        else:
            if self.useTestPicture:
                img = cv2.imread(WATCHFOLDER_PATH + path)
            else:
                img = self.bridge.imgmsg_to_cv2(self.currentImageFromStream, desired_encoding='bgr8')
            if self.IMAGE_ROTATION:
                img = cv2.rotate(img, self.IMAGE_ROTATION)
            with open(WATCHFOLDER_PATH + "face_detection_result.txt", "w") as f: # Reset the observation results
                f.write("")
            with open(WATCHFOLDER_PATH + "sketcher_result.pkl", "w") as f: # Reset the observation results
                pickle.dump("Still processing",f,protocol=2)
            cv2.imwrite(WATCHFOLDER_PATH+path, img)  
            print("Image saved in " + WATCHFOLDER_PATH + path)
            return img, cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    def analyzePicture(self, img, showAnalysis = False):
        bbox = ""
        while bbox == "":
            with open(WATCHFOLDER_PATH + "face_detection_result.txt", "r") as f:
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
            self.speak("Face too little contrast (" + str(int(faceContrast)) + "), try to get more contrast")
            return "Error: Face too little contrast, try to get more contrast", None
        
        self.speak("Looks good!")
        return "Success", img

    def speak(self, text):
        print(text)
        if self.local:
            os.system(str("say " + text))
        else:
            self.tts.say(text)

    def take_stylish_picture(self):
        self.speak("Taking a picture in 3, 2, 1. Smile!")
        success = False
        img, conv_img = self.takePicture("detect_face.jpg")
        analyzePictureResponse, _ = self.analyzePicture(conv_img, showAnalysis= True)
        if analyzePictureResponse == "Success":
            cv2.imwrite(SKETCH_FACE_FILE, img) 
            paths = "Still processing"
            print("looking for sketcher result")
            while paths == "Still processing":
                f = open(SKETCH_FACE_PATHS_FILE, "rb")   
                paths = pickle.load(f)
                print(paths)
                f.close()
                if paths == "Still processing":
                    rospy.sleep(1)
                    print("Still procesing here")
                    continue
                print("Sketcher result:", paths)
                success = True
        return success    

    ################ Running Callbacks ################

    def newImageCallback(self, img_msg):
        self.currentImageFromStream = img_msg 