#!/usr/bin/env python

"""
Seperate node caring to take high quality pictures
"""

import rospy
import os
import cv2
import numpy as np
from naoqi import ALProxy
import vision_definitions as vd
import pickle
from std_msgs.msg import Bool, Empty

################### Variables ###################
USE_MEDIA_PIPE_DIRECT = False

WATCHFOLDER_PATH = "/home/hrsa/leonao/src/leonao/watchfolder/"
SKETCH_FACE_FILE = WATCHFOLDER_PATH + "sketch_face.jpg"
SKETCH_FACE_PATHS_FILE = WATCHFOLDER_PATH + "sketcher_result.pkl"


if USE_MEDIA_PIPE_DIRECT:
    from face_detector import FaceDetector

class pictureTaker:

    """
    Class to take pictures, can be initialized via 4 different modes to accompany different test and use cases
    - Default: ALProxy - uses the AL Picture Proxy to take high resolution picutres with adjusted settings
    - TestPicture - Uses the last saved image and triggers further steps by still deleting the result files and saving the image again
    - Local - Uses the the internal laptop/pc camera to take a picture (optimized for linux, might need different settings for other OSs)
    - RosStream - Uses the low resolution ROS Stream (legacy)
    Resets the result files to indicate to the imageProcessing program that new results need to be calculated
    Saves the image to trigger the watchfolder imageProcessing

    Publishes success state via "picture_taken" topic
    """

    def __init__(self, imageSource = "ALProxy"): # imageSource "ALProxy", "TestPicture", "Local" or "RosStream"
        self.local = False
        if imageSource == "Local":
            self.local = True
        self.imageSource = imageSource
        if imageSource == "TestPicture":
            self.IMAGE_ROTATION = False
        else:
            self.IMAGE_ROTATION = cv2.ROTATE_90_COUNTERCLOCKWISE
        self.minFaceSize = 0.1
        self.minBrightness = 50
        self.maxBrightness = 200
        self.minContrast = 30
        if self.local:
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

            # Video stream
            self.camProxy = ALProxy("ALVideoDevice", self.robot_ip, 9559)


            if imageSource == "RosStream":
                self.image_sub = rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, self.newImageCallback)
            print("Picuture Taker initialized")

        self.take_picture_sub = rospy.Subscriber('/take_picture', Empty, self.take_picture_callback, queue_size=1)
        self.picture_taken_pub = rospy.Publisher('picture_taken', Bool, queue_size=1)
    
    def take_picture_callback(self, data):
        """ 
        Receiving trigger and publishing success state
        :param data: not in use
        """
        success = self.take_stylish_picture()
        self.picture_taken_pub.publish(success)

    def takePicture(self, path):
        """
        Takes a picture accordig to its initialized mode and saves it
        :param path: storage location of taken image within WATCHFOLDER_PATH

        returns BGR and RGB image (seperately)
        important return is the saving of the image though
        """
        if self.local:
            _, frame = self.camera.read()
            cv2.imwrite(WATCHFOLDER_PATH + path, frame)
            return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        else:
            if self.imageSource == "TestPicture":
                img = cv2.imread(WATCHFOLDER_PATH + path)
            elif self.imageSource == "RosStream":
                img = self.bridge.imgmsg_to_cv2(self.currentImageFromStream, desired_encoding='bgr8')
            elif self.imageSource == "ALProxy":
                # // kVGA (640x480) or k4VGA (1280x960, only with the HD camera).
                # // (Definitions are available in alvisiondefinitions.h)
                resolution = vd.k4VGA
                colorSpace = vd.kBGRColorSpace
                fps = 10
                self.camId = self.camProxy.subscribe("top_cam", resolution, colorSpace, fps)
                ## Possible values for auto exposure algorithm
                # 0: Average scene Brightness
                # 1: Weighted average scene Brightness
                # 2: Adaptive weighted auto exposure for hightlights
                # 3: Adaptive weighted auto exposure for lowlights
                self.camProxy.setCameraParameter(self.camId, vd.kCameraHFlipID, 0)
                self.camProxy.setCameraParameter(self.camId, vd.kCameraAutoExpositionID, 1)# auto exposure
                self.camProxy.setCameraParameter(self.camId, vd.kCameraExposureAlgorithmID, 2) # exposure for highlights (having the biggest effect for good images)
                self.camProxy.setCameraParameter(self.camId, vd.kCameraBrightnessID, 55) # overall brightness - 55 is default
                self.camProxy.setCameraParameter(self.camId, vd.kCameraSharpnessID, 0) # no pre-sharpening
                img = self.camProxy.getImageRemote(self.camId)
                img = np.frombuffer(img[6], dtype=np.uint8).reshape((img[1], img[0], 3))
                self.camProxy.unsubscribe(self.camId)
                print("Image taken from ALProxy")
            if self.IMAGE_ROTATION:
                img = cv2.rotate(img, self.IMAGE_ROTATION)
            with open(WATCHFOLDER_PATH + "face_detection_result.txt", "w") as f: # Reset the observation results
                f.write("")
            with open(WATCHFOLDER_PATH + "sketcher_result.pkl", "w") as f: # Reset the observation results
                pickle.dump("Still processing",f,protocol=2)
            cv2.imwrite(WATCHFOLDER_PATH+path, img)  
            print("Image saved in " + WATCHFOLDER_PATH + path)
            return img, cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    def analyzePicture(self, img):
        """

        Analyzes the image for:
        - Detecting of a face
            Done by waiting for the face detection results to be updates 
            after the new image has been saved by the former function
        - Minimal Face Brightness
        - Maximal Face Brightness
        - Minimal Face contrast

        Speaks the suggestions to improve the image

        Returns success State (with error description) and in case of success the image

        ##### Threshold values not fully optimized #####
        """
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
        """ 
        Uses ALProxy to speak, automatically setting volume to 100 and adding a pause
        :param text: string (no random list choice as in the main controller)
        """
        print(text)
        if self.local:
            os.system(str("say " + text))
        else:
            self.tts.say("\\vol=100\\" + text + "\\pau=500\\")

    def take_stylish_picture(self):
        """
        Main Function speaking, starting function to take the picture and analyze.
        Waiting until taken, analysed picture has been stylized by imageProcessing.py
        returning success (actuall paths will be read directly from file by the picture_painter)
        """
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
                f.close()
                if paths == "Still processing":
                    rospy.sleep(1)
                    print("Still procesing here")
                    continue
                #print("Sketcher result:", paths)
                success = True
                self.camProxy.unsubscribe(self.camId) # TODO: Check if unsubscribe here is the right place
        return success    

    ################ Running Callbacks ################

    def newImageCallback(self, img_msg):
        """
        Caches images from ROS Image stream to use within picture taker, when ROSImage mode is set
        :param img_smg: ros image input
        """
        self.currentImageFromStream = img_msg 



if __name__ == '__main__':
    """
    Running and spinning picture_taker node
    """
    rospy.init_node('picture_taker', anonymous=True)
    try:
        picture_taker =  pictureTaker(imageSource = "ALProxy")
        rospy.spin()

    except rospy.ROSInterruptException:
        print("picture_taker: FAILED")