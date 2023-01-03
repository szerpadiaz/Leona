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


        all_kps = []
        for p in out:
            all_kps.append(np.array([p.x, p.y]))
        all_kps = np.array(all_kps)
    
        # Denormalize, bc mediapipe outputs normlazied between 0-1
        all_kps[:, 0] *= img.shape[1]
        all_kps[:, 1] *= img.shape[0]

        # Get idcs of landmarks for transformation
        landmark_idcs = np.array([468, 473, 4, 62, 308]) # eye_left, eye_right, nose, mouth_left, mouth_right # Found by manual inspection
        landmark_kps = all_kps[landmark_idcs]

        return landmark_kps, all_kps

    def align_for_gan_input(self, img, kps, mask = None, all_kps = None, replacement_color=(255,255,255)):
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

        # Transform all Keypoints
        all_kps_tf = []
        all_kps_homog = np.hstack([all_kps, np.ones((len(all_kps), 1))])
        for kp in all_kps_homog:
            kp_tf = h @ kp
            all_kps_tf.append(kp_tf)
        all_kps_tf = np.array(all_kps_tf).astype(int)[:,:2]

        return img_aligned, kps_tf, mask_aligned, all_kps_tf


    def prepare_input_for_torch(self, img, mask, kps, all_kps):
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
        # Only needed for later processing
        item['all_kps_tf'] = all_kps

        return item

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

    def create_face_mask(self, data):
        """ 
        Creates a mask to isolate the face.
        kepoints around the face were found by manual inspection
        """
        idcs = [103, 67, 109, 10, 338, 297, 332, 284, 251, 389, 356, 447, 366, 323, 401, 435, 367, 397, 365, 379, 378, 400, 377, 152, 148, 176, 149, 150, 136, 172, 58, 215, 132, 177, 93, 137, 234, 227, 127,162, 21, 54, 103]
        outline_kps = data['all_kps_tf'][idcs]

        img_shape = np.swapaxes(np.swapaxes(data['A'].numpy()[0,:,:,:],0,2),0,1).shape

        face_mask = np.zeros(img_shape)
        face_mask = cv2.fillPoly(face_mask, [np.array(outline_kps)], (1,1,1))[:,:,0]
        data['face_mask'] = face_mask
        return face_mask

    def prepare_input(self, img):
        """
        Prepares the image for handing it to the model.
        """
        kps, all_kps = self.find_face_landmarks(img)
        img_bg_rem, mask = self.fd.remove_background(img)

        img_aligned, kps_tf, mask_aligned, all_kps_tf = self.align_for_gan_input(img_bg_rem, kps, mask=mask, all_kps=all_kps)
        data = self.prepare_input_for_torch(img_aligned, mask_aligned, kps_tf, all_kps_tf)
        return data

    def run(self, img, visualize = False):
        """
        Full pipeline to create a sketch image
        """
        input_data = self.prepare_input(img)
        face_mask = self.create_face_mask(input_data)
        out_img = self.apdrawgan.run(input_data)
        
        # Denormalize
        out_img = (out_img + 1) / 2

        if visualize:
            input_data['output'] = out_img
            input_data['masked_output'] = out_img * face_mask
            input_data['masked_output_inv'] = out_img * (face_mask*-1 + 1)
            self.visualize_inputs_outputs(input_data)

        return out_img, face_mask

    def visualize_inputs_outputs(self, data):
        import matplotlib.pyplot as plt
        rows = 5
        cols = 3
        plt.figure(figsize=(5*cols,5*rows))
        
        imgs = []
        i = 1
        for k in data.keys():
            if not (k in ['A_paths', 'center', 'all_kps_tf']):
                plt.subplot(rows,cols, i)
                try:
                    plt.imshow((np.moveaxis(data[k][0,:,:,:].numpy(), 0, -1)+1)/2)
                except:
                    plt.imshow(data[k], cmap='gray')
                plt.title(k)
                i +=1
        plt.tight_layout()
        plt.show()
