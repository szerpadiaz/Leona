# A function that watches a folder and reads new files as soon as they are there 
# Then, continues the program

import cv2
import time
import numpy as np
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import pickle

from PIL import Image

from face_detector import FaceDetector
from sketcher import Sketcher
from path_generator import Face_paths_generator, preprocessor
from local_painter import Leonao_painter

class Watcher():
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

face_paths_gen = Face_paths_generator()

class Handler(FileSystemEventHandler):

    @staticmethod
    def on_any_event(event):
        print(f"Received event - {event}")


        if event.is_directory:
            #print("Received directory event - %s.", event.src_path)
            return None

        elif event.event_type == 'modified':
            file = event.src_path
            print("Received modified event - %s.", file)

            if file[-15:] == "detect_face.jpg":
                print("Detecting face")
                img = cv2.imread(file)
                faceDetector = FaceDetector()
                bbox, _ = faceDetector.detect_face(img)
                print(bbox)
                with open(DIRECTORY_TO_WATCH + "/" + "face_detection_result.txt", "w") as f:
                    f.write(str(bbox))
            elif file[-15:] == "sketch_face.jpg":
                print("Sketching face")
                img = cv2.imread(file)
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                output_img, output_face_mask = sketcher.run(img)
                output_img = output_img*255
                ret,output_img = cv2.threshold(output_img,127,255,cv2.THRESH_BINARY)

                cv2.imwrite(DIRECTORY_TO_WATCH + "/" + "thresholded.bmp", (output_img).astype(np.uint8))

                # Preprocess the image 
                output_img = preprocessor(output_img)

                inner_sketch = output_img*output_face_mask
                outer_sketch = output_img*(1-output_face_mask)

                cv2.imwrite(DIRECTORY_TO_WATCH + "/" + "outer_sketch.bmp", (outer_sketch).astype(np.uint8))
                cv2.imwrite(DIRECTORY_TO_WATCH + "/" + "inner_sketch.bmp", (inner_sketch).astype(np.uint8))


                # Potentially simplyfing the lines more

                l_painter = Leonao_painter()
                face_outer_paths = face_paths_gen.get_face_inner_path(DIRECTORY_TO_WATCH + "/" + "outer_sketch.bmp")
                print("Outer Paths: ", face_outer_paths)
                face_inner_paths = face_paths_gen.get_face_inner_path(DIRECTORY_TO_WATCH + "/" + "inner_sketch.bmp")
                # Make sure the output here will be in CM and not in pixels (probably in the function already)
                with open(DIRECTORY_TO_WATCH + "/" + "sketcher_result.txt", "wb") as f:
                    pickle.dump(face_outer_paths, f, protocol=2)

                print("Inner Paths: ", face_inner_paths) 

                l_painter.draw(face_outer_paths, face_inner_paths)


                return None



if __name__ == '__main__':
    DIRECTORY_TO_WATCH = "/home/michael/Documents/HRS/leonao/src/leonao/watchfolder"
    w = Watcher(DIRECTORY_TO_WATCH)
    w.run()