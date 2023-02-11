#from svg_parser import get_paths_from_svg_file
from potrace_rendering import *
from Fourier_epicycles import refine_path_using_fourier_epicycles #TODO REMOVE

# Fourier Based on: https://editor.p5js.org/codingtrain/sketches/RfrZibjfL 

class Face_paths_generator():
    def __init__(self):
        self.AMP_MULTIPLIER = 2
        self.N_SKIP = 1
        self.USE_DFT = False

    # Can delete the full function
    def generate_face_paths_from_svg(self, face_image_svg, x0, y0, x1, y1):

        all_paths = []

        # Get all svg-paths from face image
        #all_paths = get_paths_from_svg_file(face_image_svg)

        # Ignore paths that are too short (they usually contain face details which we could ignore)
        svg_paths = []
        for path in all_paths:
            if(len(path) > 30):
                print( "appending path with len = ", len(path))
                svg_paths.append(path)
        
        output_paths = []
        for path in svg_paths:
            if(self.USE_DFT):
                # Refine paths using fourier
                p = refine_path_using_fourier_epicycles(path, x0, y0, x1, y1, self.AMP_MULTIPLIER, self.N_SKIP)
                print("refined path with fourier ", len(p))
                output_paths.append(p)
            else:
                output_path = []
                for i in range(0, len(path),self.N_SKIP):
                    x = self.AMP_MULTIPLIER * path[i][0]
                    y = self.AMP_MULTIPLIER * path[i][1]
                    output_path.append((x,y))
                output_paths.append(output_path)
        return output_paths

    def get_face_outer_path(self, face_image_bmp):
        turdsize = 1
        MIN_SEGMENTS_PER_PATH = 5
        MIN_DISTANCE = 10
        SIMPLE_SEGMENTS_PER_BEZIER_SEGMENT = 4

        path = get_bezier_path(face_image_bmp, turdsize)
        eliminate_short_curves(path, MIN_SEGMENTS_PER_PATH)
        eliminate_short_segments(path, MIN_DISTANCE)
        face_outer_paths = convert_to_simple_paths(path, SIMPLE_SEGMENTS_PER_BEZIER_SEGMENT)
        return face_outer_paths

    def get_face_inner_path(self, face_image_bmp):
        turdsize = 10
        MIN_SEGMENTS_PER_PATH = 3
        MAX_SEGMENTS_PER_PATH = 100
        MIN_DISTANCE = 5
        SIMPLE_SEGMENTS_PER_BEZIER_SEGMENT = 4

        path = get_bezier_path(face_image_bmp, turdsize)
        eliminate_short_curves(path, MIN_SEGMENTS_PER_PATH)
        #eliminate_long_curves(path, MAX_SEGMENTS_PER_PATH)
        eliminate_short_segments(path, MIN_DISTANCE)
        face_inner_paths = convert_to_simple_paths(path, SIMPLE_SEGMENTS_PER_BEZIER_SEGMENT)

        return face_inner_paths