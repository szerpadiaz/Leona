import math
import numpy as np
import potrace
from PIL import Image
from numpy import asarray
import cv2
import matplotlib.pyplot as plt

#####################################################
################# Supporter functions ###############
#####################################################


def erode(img, size, shape=cv2.MORPH_CROSS):
    element = cv2.getStructuringElement(shape, (2 * size + 1, 2 * size + 1),
                                       (size, size))
    return cv2.erode(img, element)

def dilate(img, size, shape=cv2.MORPH_CROSS):
    element = cv2.getStructuringElement(shape, (2 * size + 1, 2 * size + 1),
                                       (size, size))
    return cv2.dilate(img, element)


def blur(img, size):
    return cv2.GaussianBlur(img, (size,size), size/2)


def show(img, title=''):
    _,img = cv2.threshold(img,127,255,cv2.THRESH_BINARY)
    plt.figure()
    plt.imshow(img, cmap='gray')
    plt.title(title)
    return img

def print_bezier_path(path):
    # Iterate over path curves and 
    # convert it into an array of simple paths (with (x,y) pairs))
    simple_paths = []
    total_curves = 0
    for curve in path.curves:
        total_curves +=1
        total_bezier_segments = 0
        total_corner_segments = 0
        
        for segment in curve.segments:

            if segment.is_corner:
                total_corner_segments += 1
            else:
                total_bezier_segments += 1
                #print(simple_segments)
        
        print("Curve ", total_curves, " With ", \
            total_corner_segments, " corner segments" + " and ", 
            total_bezier_segments, " Bezier segments ")
    
    print("TOTAL CURVES = ", total_curves )

def get_bezier_path(image, turdsize):
    
    data = np.asarray(image, dtype=np.uint8)
    data = (data > 0).astype(np.bool_)
    bmp = potrace.Bitmap(data)

    # Trace the bitmap to a path
    turnpolicy = potrace.TURNPOLICY_MINORITY
    alphamax = 1.3333 # from 0.0 (polygon) to 1.3333 (no corners).
    opticurve = 1 
    opttolerance = 0.9
    # progress_func ?
    path = bmp.trace(turdsize, turnpolicy, alphamax, opticurve, opttolerance)

    return path    

def convert_bezier_segment_into_simple_segments(start_point, bezier_segment, points_per_segment):
    simple_segments = []
    for i in range(points_per_segment):
        t = i / points_per_segment
        
        x = (1 - t)**3 * start_point[0] + 3*(1-t)**2*t*bezier_segment.c1[0] + \
            3*(1-t)*t**2*bezier_segment.c2[0] + t**3*bezier_segment.end_point[0]

        y = (1 - t)**3 * start_point[1] + 3*(1-t)**2*t*bezier_segment.c1[1] + \
            3*(1-t)*t**2*bezier_segment.c2[1] + t**3*bezier_segment.end_point[1]

        simple_segments.append((x,y))
    return simple_segments

def convert_to_simple_paths(bezier_path, points_per_bezier_segment):
    # Iterate over path curves and 
    # convert it into an array of simple paths (with (x,y) pairs))
    simple_paths = []
    total_points = 0
    for curve in bezier_path.curves:
        simple_path = []
        simple_path.append(curve.start_point)
        previous_segment_end_point = curve.start_point
        for segment in curve.segments:
            if segment.is_corner:
                simple_path.append(segment.c)
                simple_path.append(segment.end_point)
            else:
                simple_segments = convert_bezier_segment_into_simple_segments( \
                    previous_segment_end_point, segment, points_per_bezier_segment)
                simple_path.extend(simple_segments)
            previous_segment_end_point = segment.end_point
        
        simple_paths.append(simple_path)
        total_points += len(simple_path)
    
    print("TOTAL PATHS = ",  len(simple_paths), " ; TOTAL POINTS = ", total_points)
    return simple_paths
def eliminate_out_of_range(paths, min, max):
    total_new_points = 0
    new_paths = []
    for path in paths:
        new_path = []
        for point in path:
            x_in_range = min[0] <= point[0] <= max[0]
            y_in_range = min[1] <= point[1] <= max[1]
            if x_in_range and y_in_range:
                new_path.append(point)
                total_new_points += 1
            else:
                if len(new_path) > 1:
                    new_paths.append(new_path)
                new_path = []

        if len(new_path) > 1:
            new_paths.append(new_path)

    print("TOTAL PATHS = ",  len(new_paths), " ; TOTAL POINTS = ", total_new_points)

    return new_paths

def normalize_paths(simple_paths, top_left_point, bottom_right_point):
    height = bottom_right_point[1] - top_left_point[1]
    width = bottom_right_point[0] - top_left_point[0]

    x_offset = top_left_point[0]
    y_offset = top_left_point[1]

    new_paths = []
    for subpath in simple_paths:
        new_path = []
        #print("Paths points ", len(subpath))
        for x, y in subpath:
            new_x = (x - x_offset) / width
            new_y = (y - y_offset) / height
            new_path.append((new_x, new_y))
        new_paths.append(new_path)
    return new_paths

##########################################################################################
################## Face paths generator class ###########################################
##########################################################################################


class Face_paths_generator():
    def __init__(self, face_info):
        self.top_left_point = face_info["top_left_point"]
        self.bottom_right_point = face_info["bottom_right_point"]
        self.outer_img_file = face_info["outer"]
        self.inner_img_file = face_info["inner"]
        
        #face_sketch_file = "../test_imgs/monalisa_sketch.bmp"
        #face_mask_file = "../test_imgs/monalisa_sketch_mask.png"
        #face_inner_sketch_file = "../test_imgs/monalisa_sketch_outer_2.bmp"
        #face_outer_sketch_file = "../test_imgs/monalisa_sketch_inner_2.bmp"
        #
        ## Read face_sketch and face_mask
        #output_img = cv2.imread(face_sketch_file, cv2.IMREAD_GRAYSCALE)
        #_, output_img = cv2.threshold(output_img, 127, 255, cv2.THRESH_BINARY)
        #output_face_mask = cv2.imread(face_mask_file, cv2.IMREAD_GRAYSCALE)
        #_, output_face_mask = cv2.threshold(output_face_mask, 127, 255, cv2.THRESH_BINARY)
        #
        ## Generate face_outer_sketch
        #outer_sketch = cv2.bitwise_or(output_img, output_face_mask)
        #cv2.imwrite(face_outer_sketch_file, (outer_sketch).astype(np.uint8))
        #
        ## Generate face_inner_sketch_file
        #output_face_mask_inverted = cv2.bitwise_not(output_face_mask)
        #inner_sketch = cv2.bitwise_or(output_img, output_face_mask_inverted)
        #cv2.imwrite(face_inner_sketch_file, (inner_sketch).astype(np.uint8))
        #
        ###cv2.imshow("output_img", output_img)
        ##cv2.imshow("output_face_mask", output_face_mask)
        ##cv2.imshow("outer_sketch", outer_sketch)
        ##cv2.imshow("inner_sketch", inner_sketch)
        ##cv2.waitKey(0)

    def get_face_outer_path(self):
        img = cv2.imread(self.outer_img_file, cv2.IMREAD_GRAYSCALE)
        _, img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

        img = erode(img,3, cv2.MORPH_ELLIPSE)
        img = dilate(img,3, cv2.MORPH_ELLIPSE)

        #img = erode(img,5, cv2.MORPH_CROSS)
        #img = dilate(img,5, cv2.MORPH_CROSS)

        #cv2.imshow("face_outer", img)
        #cv2.waitKey(0)

        turdsize = 2
        SIMPLE_SEGMENTS_PER_BEZIER_SEGMENT = 4 #6, 8

        print("Generating face_outer_paths")
        path = get_bezier_path(img, turdsize)
        face_outer_paths = convert_to_simple_paths(path, SIMPLE_SEGMENTS_PER_BEZIER_SEGMENT)
        face_outer_paths = eliminate_out_of_range(face_outer_paths, self.top_left_point, self.bottom_right_point)
        return face_outer_paths

    def get_face_inner_path(self):
        img = cv2.imread(self.inner_img_file, cv2.IMREAD_GRAYSCALE)
        _, img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

        #img = erode(img,1, cv2.MORPH_ELLIPSE)
        #img = dilate(img, 1, cv2.MORPH_ELLIPSE)

        img = erode(img, 2, cv2.MORPH_CROSS)
        img = dilate(img, 2, cv2.MORPH_CROSS)
 
        #cv2.imshow("face_outer", img)
        #cv2.waitKey(0)

        turdsize = 8
        SIMPLE_SEGMENTS_PER_BEZIER_SEGMENT = 8 #8, 16

        print("Generating face_inner_paths")
        path = get_bezier_path(img, turdsize)
        face_inner_paths = convert_to_simple_paths(path, SIMPLE_SEGMENTS_PER_BEZIER_SEGMENT)
        face_inner_paths = eliminate_out_of_range(face_inner_paths, self.top_left_point, self.bottom_right_point)
        return face_inner_paths

    def normalize_face_path(self, face_path):
        face_path_normalized = normalize_paths(face_path, self.top_left_point, self.bottom_right_point)
        return face_path_normalized
##########################################################################################
################## preproccessor  ###########################################
##########################################################################################

def preprocessor(img):
    img = erode(img,2, cv2.MORPH_ELLIPSE)
    #img = dilate(img,3, cv2.MORPH_ELLIPSE)
    img = blur(img, 5)
    _,img = cv2.threshold(img,127,255,cv2.THRESH_BINARY)
    return img