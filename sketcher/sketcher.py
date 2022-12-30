# Imports
import numpy as np
import cv2
import mediapipe
import scipy
import torch
import torchvision
import os

import sys
sys.path.append('../')
from face_detector.face_detector import FaceDetector

sys.path.append('./apdrawgan_adjusted')
from apdrawgan_adjusted import apdrawgan

class Sketcher:
    def __init__(self):
        """
        Initialize Object and hand over proper directories
        """
        self.fd = FaceDetector()
        base_path = os.path.dirname(apdrawgan.__file__)
        self.apdrawgan = apdrawgan.APDrawGAN(base_path)

    def find_face_landmarks(self, img):
        """
        Find the 5 facial landmarks eye_l, eye_r, nose, mouth_l, mouth_r
        Based on mediapipe face_mesh
        """
        fm = mediapipe.solutions.face_mesh.FaceMesh(static_image_mode=True,
                                                    max_num_faces=1,
                                                    refine_landmarks=True,
                                                    min_detection_confidence=0.5)
        # Find Landmarks
        results = fm.process(img)
        out = results.multi_face_landmarks[0].landmark

        # These were found by manual inspection
        eye_left = [out[468].x, out[468].y]      #[(out[130].x + out[133].x)/2, (out[130].y + out[133].y)/2]
        eye_right = [out[473].x, out[473].y]     #[(out[359].x + out[398].x)/2, (out[359].y + out[398].y)/2]
        nose = [out[4].x, out[4].y]            #[out[4].x, out[4].y]
        mouth_left = [out[62].x, out[62].y]     #[out[186].x, out[186].y]
        mouth_right = [out[308].x, out[308].y]   #[out[292].x, out[292].y]

        # Denormalize as mediapipe returns normalized coordinates in Interval [0,1]
        kps = np.array([eye_left, eye_right, nose, mouth_left, mouth_right])
        kps[:, 0] *= img.shape[1]
        kps[:, 1] *= img.shape[0]

        return kps

    def align_for_gan_input(self, img, kps, mask=None, replacement_color=(255,255,255)):
        """
        Aligns points just like in the matlab script under /preprocess/example/face_align_512.m
        It's not 100% the same bc matlab calculates transformation different, but should be sufficient
        Outputs images of fixed size (512 x 512)
        :param img: input image
        :param kps: 5 keypoints in order: eye left, eye right, nose, mouth left, mouth right
        :return img_out: Rotated, Translated, Scaled and cropped image
        """
        # Setup images size and baseline corresepondence values
        img_size = (512,512)
        base_landmarks_5 = np.array([[180,230],[300,230], [240,301], [186,365.6], [294,365.6]])
        base_landmarks_5 = (base_landmarks_5-240)/560 * 512 + 256

        # Estimate homography matrix (affine transform) from correspondences
        h, _ = cv2.estimateAffinePartial2D(kps, base_landmarks_5)
        h = np.vstack([h,[0,0,1]]) # Add vector to create valid Homography matrix

        # transform image and background mask
        img_aligned = cv2.warpPerspective(img, h, img_size, borderMode = cv2.BORDER_CONSTANT, borderValue = replacement_color)
        
        mask_aligned = None
        if mask is not None:
            if (mask.dtype == bool):
                # Transform mask to be uint8 and 3 channels to work with warpPerspective
                mask = 255*np.dstack([mask, mask, mask]).astype(np.uint8)
            mask_aligned = cv2.warpPerspective(mask, h, img_size, borderMode = cv2.BORDER_CONSTANT, borderValue = (255,255,255))
            # Transform back to 1 channel bool
            mask_aligned = mask_aligned[:,:,0] < 127
        
        # Transform keypoints to algined image
        kps_homog = np.hstack([kps, np.ones((len(kps), 1))])
        kps_tf = []
        for kp in kps_homog:
            kp_tf = h @ kp
            kps_tf.append(kp_tf)
        kps_tf = np.array(kps_tf).astype(int)[:,:2]

        return img_aligned, kps_tf, mask_aligned

    def normalize(self, img):
        """
        Normalizes Image to be between -1 and 1
        """
        if (img.dtype == np.uint8):
            return (2*img.astype(np.float64)/255 - 1)
        else:
            if (np.max(img) > 1.0) or (np.min(img) < 0):
                return 2*(img - np.min(img))/(np.max(img)-np.min(img)) - 1
            else:
                return 2*img - 1

    def prepare_input_for_torch(self, img, mask, kps):
        """
        Creates the necessary cropped images etc for APDrawGAN input
        Setup like authors of APDrawGAN do it
        """
        RATIO = 2
        EYE_H = 80      # 40 * RATIO
        EYE_W = 112     # 56 * RATIO
        NOSE_H = 96     # 48 * RATIO
        NOSE_W = 96     # 48 * RATIO
        MOUTH_H= 80     # 40 * RATIO
        MOUTH_W= 128    # 64 * RATIO

        item = dict()

        # Calculate Center point of mouth
        mouth = (kps[3,:] + kps[4,:])//2
        center = kps[:4].copy()
        center[3,:] = mouth
        center = np.array([[kps[0,0], kps[0,1]-4*RATIO], [kps[1,0], kps[1,1]-4*RATIO], [kps[2,0], kps[2,1]-NOSE_H/2+16*RATIO], [mouth[0],mouth[1]]],dtype=int)

        # Create a region mask that masks out eyes, nose mouth and background
        region_mask = np.ones((img.shape[0], img.shape[1], 1))
        region_mask[center[0,1] - EYE_H//2:center[0,1] + EYE_H//2, center[0,0] - EYE_W//2:center[0,0] + EYE_W//2,:] = 0
        region_mask[center[1,1] - EYE_H//2:center[1,1] + EYE_H//2, center[1,0] - EYE_W//2:center[1,0] + EYE_W//2,:] = 0
        region_mask[center[2,1] - NOSE_H//2:center[2,1] + NOSE_H//2, center[2,0] - NOSE_W//2:center[2,0] + NOSE_W//2,:] = 0
        region_mask[center[3,1] - MOUTH_H//2:center[3,1] + MOUTH_H//2, center[3,0] - MOUTH_W//2:center[3,0] + MOUTH_W//2,:] = 0

        # Mask image to create bg and hair images
        hair_A = ((img/255)) * np.dstack([region_mask[:,:,0],region_mask[:,:,0],region_mask[:,:,0]]) * np.dstack([mask, mask, mask])
        bg_A = (img/255/2 + 0.5) * np.dstack([~mask, ~mask, ~mask])
        
        # Normalize and assign
        item['A'] = self.normalize(img)
        item['center'] = center
        item['eyel_A'] = self.normalize(img[center[0,1] - EYE_H//2:center[0,1] + EYE_H//2, center[0,0] - EYE_W//2:center[0,0] + EYE_W//2,:])
        item['eyer_A'] = self.normalize(img[center[1,1] - EYE_H//2:center[1,1] + EYE_H//2, center[1,0] - EYE_W//2:center[1,0] + EYE_W//2,:])
        item['nose_A'] = self.normalize(img[center[2,1] - NOSE_H//2:center[2,1] + NOSE_H//2, center[2,0] - NOSE_W//2:center[2,0] + NOSE_W//2,:])
        item['mouth_A'] = self.normalize(img[center[3,1] - MOUTH_H//2:center[3,1] + MOUTH_H//2, center[3,0] - MOUTH_W//2:center[3,0] + MOUTH_W//2,:])
        item['hair_A'] = self.normalize(hair_A)
        item['bg_A'] = self.normalize(bg_A)
        item['mask'] = region_mask
        item['mask2'] = mask.astype(np.float64)

        # Convert to Torch Tensors
        item['A'] = (torch.from_numpy(np.expand_dims(np.moveaxis(item['A'], -1, 0), 0))).float()
        item['mask'] = (torch.from_numpy(np.expand_dims(np.moveaxis(item['mask'], -1, 0), 0))).float()
        item['mask2'] = (torch.from_numpy(np.expand_dims(np.expand_dims(np.moveaxis(item['mask2'], -2, 0), 0),0))).float()
        item['center'] = torch.from_numpy(np.expand_dims(item['center'],0))
        item['A_paths'] = ['']
        item['hair_A'] = (torch.from_numpy(np.expand_dims(np.moveaxis(item['hair_A'], -1, 0), 0))).float()
        item['bg_A'] = (torch.from_numpy(np.expand_dims(np.moveaxis(item['bg_A'], -1, 0), 0))).float()
        item['eyel_A'] = (torch.from_numpy(np.expand_dims(np.moveaxis(item['eyel_A'], -1, 0), 0))).float()
        item['eyer_A'] = (torch.from_numpy(np.expand_dims(np.moveaxis(item['eyer_A'], -1, 0), 0))).float()
        item['nose_A'] = (torch.from_numpy(np.expand_dims(np.moveaxis(item['nose_A'], -1, 0), 0))).float()
        item['mouth_A'] = (torch.from_numpy(np.expand_dims(np.moveaxis(item['mouth_A'], -1, 0), 0))).float()
        
        return item


    def prepare_input(self, img):
        """
        Prepares the image for handing it to the model.
        """
        kps = self.find_face_landmarks(img)
        img_bg_rem, mask = self.fd.remove_background(img)

        img_aligned, kps_tf, mask_aligned = self.align_for_gan_input(img_bg_rem, kps, mask=mask)
        data = self.prepare_input_for_torch(img_aligned, mask_aligned, kps_tf)
        return data

    def run(self, img, visualize_inputs = False):
        """
        Full pipeline to create a sketch image
        """
        input_data = self.prepare_input(img)
        out_img = self.apdrawgan.run(input_data)
        #out_img = ((out_img+1) / 2 * 255).astype(np.uint8)
        out_img = (out_img + 1) / 2

        if visualize_inputs:
            self.visualize_inputs(input_data)
        return out_img

    def visualize_inputs(self, data):
        import matplotlib.pyplot as plt
        rows = 3
        cols = 3
        plt.figure(figsize=(15,15))
        
        imgs = []
        i = 1
        for k in data.keys():
            if not (k in ['A_paths', 'center']):
                plt.subplot(rows,cols, i)
                plt.imshow(np.moveaxis(data[k][0,:,:,:].numpy(), 0, -1))
                plt.title(k)
                i +=1
        plt.tight_layout()
        plt.show()
