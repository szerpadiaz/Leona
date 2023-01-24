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

def get_bezier_path(bmp_file, turdsize):
    # Convert bmp file into bitmap
    image = Image.open(bmp_file)
    #thresh = 200
    #fn = lambda x : 255 if x > thresh else 0
    #image = image.convert('L').point(fn, mode='1')
    image = image.convert('1')
    data = asarray(image)
    bmp = potrace.Bitmap(data)

    # Trace the bitmap to a path
    turnpolicy = potrace.TURNPOLICY_MINORITY
    alphamax = 1.3333 # from 0.0 (polygon) to 1.3333 (no corners).
    opticurve = 1 
    opttolerance = 0.5
    # progress_func ?
    path = bmp.trace(turdsize, turnpolicy, alphamax, opticurve, opttolerance)

    return path    

def eliminate_long_curves(path, max_segments_per_curve):
    new_curves = []
    total_initial_curves = len(path.curves)
    for curve in path.curves:
        total_segments = len(curve.segments)
        if(total_segments < max_segments_per_curve):
            new_curves.append(curve)

    path.curves = new_curves
    total_curves_eliminated = total_initial_curves - len(path.curves)
    print("Eliminated ", total_curves_eliminated, " curves (from ", total_initial_curves, ")")
   

def eliminate_short_curves(path, min_segments_per_curve):
    new_curves = []
    total_initial_curves = len(path.curves)
    for curve in path.curves:
        total_segments = len(curve.segments)
        if(total_segments > min_segments_per_curve):
            new_curves.append(curve)

    path.curves = new_curves
    total_curves_eliminated = total_initial_curves - len(path.curves)
    print("Eliminated ", total_curves_eliminated, " curves (from ", total_initial_curves, ")")
    
def eliminate_short_segments(path, min_distance):
    total_eliminated_segments = 0
    total_segments = 0
    for curve in path.curves:
        total_segments += len(curve.segments)
        previous_segment_end_point = curve.start_point
        segment_index = 0
        new_segments = []
        for segment in curve.segments:
            distance_to_prev_point = math.dist(segment.end_point, previous_segment_end_point)
            #print(distance_to_prev_point)
            if(distance_to_prev_point < min_distance):
                previous_segment_end_point = curve.segments[segment_index - 1].end_point
                #curve.segments.pop(segment_index)
                total_eliminated_segments += 1
            else:
                new_segments.append(segment)
                previous_segment_end_point = segment.end_point
            
            segment_index += 1
        curve.segments = new_segments

    print("Eliminated ", total_eliminated_segments, " segments (from ", total_segments, ")")
    
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

##########################################################################################
################## Face paths generator class ###########################################
##########################################################################################


class Face_paths_generator():
    def __init__(self):
        self.AMP_MULTIPLIER = 2
        self.N_SKIP = 1
        self.USE_DFT = False

    def get_face_outer_path(self, face_image_bmp):
        turdsize = 400
        MIN_SEGMENTS_PER_PATH = 100
        MIN_DISTANCE = 10
        SIMPLE_SEGMENTS_PER_BEZIER_SEGMENT = 4

        path = get_bezier_path(face_image_bmp, turdsize)
        eliminate_short_curves(path, MIN_SEGMENTS_PER_PATH)
        eliminate_short_segments(path, MIN_DISTANCE)
        face_outer_path = convert_to_simple_paths(path, SIMPLE_SEGMENTS_PER_BEZIER_SEGMENT)
        return face_outer_path

    def get_face_inner_path(self, face_image_bmp):
        # Get face_inner_path
        turdsize = 50
        MIN_SEGMENTS_PER_PATH = 10
        MAX_SEGMENTS_PER_PATH = 100
        MIN_DISTANCE = 4
        SIMPLE_SEGMENTS_PER_BEZIER_SEGMENT = 4

        path = get_bezier_path(face_image_bmp, turdsize)
        eliminate_short_curves(path, MIN_SEGMENTS_PER_PATH)
        eliminate_long_curves(path, MAX_SEGMENTS_PER_PATH)
        eliminate_short_segments(path, MIN_DISTANCE)
        face_inner_path = convert_to_simple_paths(path, SIMPLE_SEGMENTS_PER_BEZIER_SEGMENT)

        return face_inner_path

##########################################################################################
################## preproccessor  ###########################################
##########################################################################################

def preprocessor(img):
    img = erode(img,3, cv2.MORPH_ELLIPSE)
    #img = dilate(img,3, cv2.MORPH_ELLIPSE)
    img = blur(img, 13)
    _,img = cv2.threshold(img,127,255,cv2.THRESH_BINARY)
    return img