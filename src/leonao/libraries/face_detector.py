import mediapipe
import matplotlib.pyplot as plt
import cv2
import numpy as np

class FaceDetector:
    """
    Wrapper Object for Blazeface
    """
    def __init__(self, threshold = 0.5):
        self.face_detector = mediapipe.solutions.face_detection.FaceDetection(model_selection=1, min_detection_confidence=threshold)
        self.bg_segmentor =  mediapipe.solutions.selfie_segmentation.SelfieSegmentation(model_selection=0)

    def detect_face(self, img):
        """
        :param img: input image
        :return bbox: Bounding Box coordinates in (x,y, w, h)
        :return kps: Facial Keypoints: 6 points, each with [x,y]
        """
        results = self.face_detector.process(img)
        if not results:
            print('[FaceDetection][INFO]: No Face found.')
            return None, None

        if results.detections == None:
            print('[FaceDetection][INFO]: No Face found.')
            return None, None

        # Get Data from 
        height, width, channels = img.shape
        bb_x = results.detections[0].location_data.relative_bounding_box.xmin
        bb_y = results.detections[0].location_data.relative_bounding_box.ymin
        bb_w = results.detections[0].location_data.relative_bounding_box.width
        bb_h = results.detections[0].location_data.relative_bounding_box.height
        
        x = int(bb_x*width)
        y = int(bb_y*height)
        w = int(width*bb_w)
        h = int(height*bb_h)
        
        bbox = np.array([x,y,w,h])
        kps = list()

        keypoints = results.detections[0].location_data.relative_keypoints
        for kp in keypoints:
            kp_x = int(kp.x*width)
            kp_y = int(kp.y*height)
            kps.append([kp_x, kp_y])
        kps = np.array(kps)

        return bbox, kps

    def pad_crop_detection(self, img, bbox, keypoints = None, x_ratio = (0.4, 0.4), y_ratio=(0.55, 0.25)):
        """
        # Sum of x_ratio should be same as sum of y_ratio
        Extend Bounding box and output image to include forehead, ears, neck and be square
        """
        img_height, img_width, img_c = img.shape

        x1, y1, b_width, b_height = bbox
        x_mid = x1 + b_width//2
        y_mid = y1 + b_height//2
        x2 = x1 + b_width
        y2 = y1 + b_height

        # make box square based on longer edge
        if b_width > b_height:
            b_height = b_width
            y1 = y_mid - b_height//2
            y2 = y_mid + b_height//2
        elif b_width < b_height:
            b_width = b_height
            x1 = x_mid - b_width//2
            x2 = x_mid + b_width//2
        
        # extend bbounding box
        x1_pad = x1 - int(x_ratio[0] * b_width)
        x2_pad = x2 + int(x_ratio[1] * b_width)
        y1_pad = y1 - int(y_ratio[0] * b_height)
        y2_pad = y2 + int(y_ratio[1] * b_height)
        bbox_pad = np.array([x1_pad, y1_pad, x2_pad - x1_pad, y2_pad - y1_pad])
        # Validate new coordinates and extend image if necessary
        borders = np.array([x1_pad, img_width - x2_pad, y1_pad, img_height - y2_pad])
        if ((borders < 0).any()):
            distance = np.abs(np.min(borders))

            img_enlarged = 255*np.ones((img_height+2*distance, img_width + 2*distance, 3), np.uint8)
            img_enlarged[distance:-distance, distance:-distance] = img
            img = img_enlarged

            x1_pad += distance
            x2_pad += distance
            y1_pad += distance
            y2_pad += distance

            # adjust keypoints if necessary
            keypoints += distance
        
        img_crop = img[y1_pad:y2_pad, x1_pad:x2_pad]
        # recalculate keypoints
        keypoints[:,0] = keypoints[:,0] - x1_pad
        keypoints[:,1] = keypoints[:,1] - y1_pad

        return img_crop, keypoints, bbox_pad

    def remove_background(self, img, replacement_color = (255,255,255), threshold = 0.8):
        """
        Removes the background from an image and replaces it with specified color (like in zoom)
        :param img: Input Image, works with any size
        :param replacement_color: new background color, RGB tuple 
        :param threshold: Threshold for creating binary mask from probability map from model

        """
        result = self.bg_segmentor.process(img)
        mask = result.segmentation_mask < threshold
        out_img = img.copy()
        out_img[mask] = replacement_color

        return out_img
        
    def detect_face_and_crop(self, img):
        """
        Combination of Detection and Cropping functions
        """
        bbox, kps = self.detect_face(img)
        if bbox is None:
            return False
        
        img_crop, kps, bbox = self.pad_crop_detection(img, bbox, kps)
        
        return img_crop, kps, bbox


    def draw_detection(self, img, bbox, kps=[], plot = True):
        """
        Helper function to draw detections
        :param plot: Plot using matplotlib if True
        """
        out = img.copy()
        cv2.rectangle(out, bbox[:2], bbox[:2]+bbox[2:], (255,0,255), 2)
        for num, kp in enumerate(kps):
            cv2.circle(out, (kp[0], kp[1]), radius=3, color=((num*126)%255,(num*50)%255, (num*85)%255) , thickness=-1)
    
        if plot:
            plt.imshow(out)
            plt.title('[Face Detector]')

        return out

    def detect_face_and_draw(self, img, plot = True):
        """
        Detect Face and draw on image
        """
        bbox, kps = self.detect_face(img)
        if (bbox == None).any():
            return img, None, None
        out = self.draw_detection(img, bbox, kps)
        return out, bbox, kps

    def detect_crop_bg_removal(self, img, replacement_color=(255,255,255), threshold=0.8):
        img_crop, kps, bbox = self.detect_face_and_crop(img)
        img_crop_bgrem = self.remove_background(img_crop, replacement_color=replacement_color, threshold=threshold)
        return img_crop_bgrem, kps
