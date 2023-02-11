"""
# Image Processing

# A function that watches a folder and reads new files as soon as they are there 
# Then, continues the program

## Face Detection - triggered through a new file called "detect_face.jpg"
# 1. Read the image
# 2. Detect the face
# 3. Write the bounding box to a file called "results.txt"

## Sketch Creation
# 1. Read the image
# 2. Detect the face (done again here to be more robust for e.g. tests)
# 3. Create a sketch using the APDrawingGAN
"""
import cv2
import time
import numpy as np
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import pickle
from copy import deepcopy

from PIL import Image

from face_detector import FaceDetector
from sketcher import Sketcher
from path_generator import Face_paths_generator, preprocessor
from local_painter import Leonao_painter

def normalizePaths(input_paths):
    # Make sure the output here will be in 0-1 and not in pixels (probably in the function already)
    # Scale all values in face_inner_paths and face_outer_paths to 0-1

    # Get the maximum value of tuples in lists of lists in face_inner_paths and face_outer_paths
    # Divide all values in face_inner_paths and face_outer_paths by the maximum value
    x_max = 0.0
    x_min = 512.0
    y_max = 0.0
    y_min = 512.0

    for paths in input_paths: # TODO: Not efficient, maybe needs to be changed
        for list in paths:
            for tuples in list:
                (x,y) = tuples
                if x > x_max:
                    x_max = x
                if x < x_min:
                    x_min = x
                if y > y_max:
                    y_max = y
                if y < y_min:
                    y_min = y
    
    print("X max: ", x_max)
    print("X min: ", x_min)
    print("Y max: ", y_max)
    print("Y min: ", y_min)

    if x_max - x_min > y_max - y_min:
        longest_side = x_max - x_min
    else:
        longest_side = y_max - y_min
    all_paths_list = input_paths
    for p, paths in enumerate(all_paths_list):
        for i, list in enumerate(paths):
            for j, tuples in enumerate(list):
                (x,y) = tuples
                all_paths_list[p][i][j] = [(x - x_min)/longest_side, (y - y_min)/longest_side]
    
    return all_paths_list

def get_bb_points_ratio43(mask, y_pad = (0.8, 0.3)):
    """
    Finds extended upper left and lower right points from input face mask.
    Allows for precise cutting of the face to ensure maximum face area.
    Ratio of BBox is ~ 4:3
    :param mask: output mask from sketcher
    :param y_pad: Paddings added to upper (idx 0) and lower (idx 1) edge of mask
    :return: dictionary containing upper left and lower right points coords: [x,y]
    """
    idcs_y, idcs_x = np.where(mask!=0)
    x1 = np.min(idcs_x)
    x2 = np.max(idcs_x)
    y1 = np.min(idcs_y)
    y2 = np.max(idcs_y)

    x_mid = (x1 + x2)//2
    y_mid = (y1 + y2)//2
    b_width = x2 - x1
    b_height = y2 - y1

    img_height, img_width = mask.shape

    # make box square based on longer edge
    if b_width > b_height:
        b_height = b_width
        y1 = y_mid - b_height//2
        y2 = y_mid + b_height//2
    elif b_width < b_height:
        b_width = b_height
        x1 = x_mid - b_width//2
        x2 = x_mid + b_width//2
    
    # Extended y coords
    y1_p = y_mid - int((b_height/2)*(1+y_pad[0]))
    y2_p = y_mid + int((b_height/2)*(1+y_pad[1]))
    
    # Makes sure new coords stay in image boundaries
    if y1_p < 0:
        y1_p = 0
    if y2_p > img_height:
        y2_p = img_height - 1

    p_height = y2_p - y1_p
    p_width = int(p_height/4*3)

    # extended x coords
    x1_p = x_mid - p_width//2
    x2_p = x_mid + p_width//2
    
    # Makes sure new x coords stay in image boundaries
    if x1_p < 0:
        x1_p = 0
    if x2_p > img_width:
        x2_p = img_width - 1

    return [x1_p, y1_p], [x2_p, y2_p]


class Watcher():
    # Watches the folder for changes 
    # Trigggers the handler on change

    def __init__(self, DIRECTORY_TO_WATCH):
        self.observer = Observer()
        self.DIRECTORY_TO_WATCH = DIRECTORY_TO_WATCH
        print("Watching: " + self.DIRECTORY_TO_WATCH)

    def run(self):
        event_handler = Handler()
        self.observer.schedule(event_handler, self.DIRECTORY_TO_WATCH, recursive=True)
        self.observer.start()
        try:
            while True:
                time.sleep(5) # This is not the polling time, so 5s is not slowing us down
        except:
            self.observer.stop()
            print("Error")

        self.observer.join()

sketcher = Sketcher()

class Handler(FileSystemEventHandler):
    # Handles the event when a file is changed
    # On image modification, the filename is checked and approprite action is taken (see below)
    ## Input and output is coming through files saved to the watchfolder, the function itself has no return value

    @staticmethod
    def on_any_event(event):
        print(f"Received event - {event}")


        if event.is_directory:
            return None

        elif event.event_type == 'modified':
            file = event.src_path
            print("Received modified event - %s.", file)
            if file[-15:] == "detect_face.jpg":
                print("Detecting face")

                # This sleep is as a workaround for the fact that the file 
                #   is not yet fully written when the event is triggered.
                #   Timing could be further reduced
                time.sleep(1) 

                img = cv2.imread(file)
                if img is None:
                    img = cv2.imread(file)
                    if img is None:
                        print("Tried to read the file twice but failed") 
                        # After introducing the sleep above this has not happnend yet
                faceDetector = FaceDetector()
                bbox, _ = faceDetector.detect_face(img)
                print("Face detected with bounding box:" + bbox)
                with open(DIRECTORY_TO_WATCH + "/" + "face_detection_result.txt", "w") as f:
                    f.write(str(bbox))
            elif file[-15:] == "sketch_face.jpg":
                f = open(DIRECTORY_TO_WATCH + "/" + "sketcher_result.pkl", "rb")   
                paths = pickle.load(f)
                f.close()

                # Need to check here if there are no paths yet, because the event is triggered twice
                # TODO: Investigate why the event is triggered twice and if we just need to listen to the second event as then the file might be fully written
                if paths == "Still processing": 
                    print("Sketching face")
                    # The following sleep is a workaround for the fact that the file 
                    #   is not yet fully written when the event is triggered.
                    #   Timing could be further reduced
                    time.sleep(1)
                    img = cv2.imread(file)

                    if img is None:
                        img = cv2.imread(file)
                        if img is None:
                            print("Tried to read the file twice but failed")
                            # After introducing the sleep above this has not happnend yet
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

                    # Sketching face
                    output_img, output_face_mask = sketcher.run(img)

                    top_left_point, bottom_right_point = get_bb_points_ratio43(output_face_mask, y_pad=(0.8, 0.05))
                    
                    ## Eroding face mask to avoid double edges around face (due to splitting of face and background)
                    # Calculate iterations based on image size
                    shape = output_img.shape
                    iterations = int(shape[1]/64)

                    kernel = np.ones((3,3),np.uint8)
                    output_face_mask = cv2.erode(output_face_mask, kernel, iterations = iterations)

                    # Convert face_sketch and face_mask to binary
                    output_img = output_img*255 
                    _, output_img = cv2.threshold(output_img, 127, 255, cv2.THRESH_BINARY)
                    
                    # Generate inner_sketch and outer_sketch using mask
                    inner_sketch = output_img * output_face_mask + (1-output_face_mask)*255
                    outer_sketch = output_img * (1 - output_face_mask) + (output_face_mask*255)

                    # Store generated files (for debugging and insight but also for the next step)
                    thresholded_sketch_file = DIRECTORY_TO_WATCH + "/" + "thresholded.bmp"
                    outer_sketch_file = DIRECTORY_TO_WATCH + "/" + "outer_sketch.bmp"
                    inner_sketch_file = DIRECTORY_TO_WATCH + "/" + "inner_sketch.bmp"
                    cv2.imwrite(thresholded_sketch_file, (output_img).astype(np.uint8))
                    cv2.imwrite(outer_sketch_file, (outer_sketch).astype(np.uint8))
                    cv2.imwrite(inner_sketch_file, (inner_sketch).astype(np.uint8))
                    
                    face_info = {"inner": inner_sketch_file, "outer" : outer_sketch_file,  "top_left_point" : top_left_point, "bottom_right_point" : bottom_right_point}
                    
                    # Generate face paths
                    face_paths_gen = Face_paths_generator(face_info)
                    face_outer_paths = face_paths_gen.get_face_outer_path()
                    face_inner_paths = face_paths_gen.get_face_inner_path()
                    face_inner_paths_norm = face_paths_gen.normalize_face_path(face_inner_paths)
                    face_outer_paths_norm = face_paths_gen.normalize_face_path(face_outer_paths)
                    all_paths = {"inner": face_inner_paths_norm, "outer": face_outer_paths_norm}
                    print("All Paths size: ", (len(all_paths["inner"]) + len(all_paths["outer"])))
                    print("All Path points amount: ", sum([len(list) for list in all_paths["inner"]]) + sum([len(list) for list in all_paths["outer"]]))
                    with open(DIRECTORY_TO_WATCH + "/" + "sketcher_result.pkl", "wb") as f:
                        pickle.dump(all_paths, f, protocol=2)

                    
                    # Visualize the paths
                    l_painter = Leonao_painter()
                    face_outer_paths_original = deepcopy(face_outer_paths)
                    face_inner_paths_original = deepcopy(face_inner_paths)
                    l_painter.draw(face_outer_paths_original, face_inner_paths_original)

                    return None
                else:
                    print("Not sketching as second change")



if __name__ == '__main__':
    DIRECTORY_TO_WATCH = "/home/hrsa/leonao/src/leonao/watchfolder"
    w = Watcher(DIRECTORY_TO_WATCH)
    w.run()