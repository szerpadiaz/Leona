# A function that watches a folder and reads new files as soon as they are there 
# Then, continues the program

import cv2
import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

from face_detector import FaceDetector
from sketcher import Sketcher

class Watcher():
    def __init__(self, DIRECTORY_TO_WATCH):
        self.observer = Observer()
        self.DIRECTORY_TO_WATCH = DIRECTORY_TO_WATCH

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

class Handler(FileSystemEventHandler):

    def __init__(self) -> None:
        super().__init__()
        self.sketcher = Sketcher()

    @staticmethod
    def on_any_event(self, event):
        if event.is_directory:
            return None

        elif event.event_type == 'modified':
            file = event.src_path
            print("Received modified event - %s.", file)
            if file == "detect_face.jpg":
                img = cv2.imread(file)
                faceDetector = FaceDetector()
                bbox, _ = faceDetector.detect_face(img)
                print(bbox)
                with open(DIRECTORY_TO_WATCH + "/" + "result.txt", "w") as f:
                    f.write(str(bbox))
            elif file == "sketch_face.jpg":
                img = cv2.imread(file)
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                output_img, output_face_mask = self.sketcher.run(img)
                


if __name__ == '__main__':
    DIRECTORY_TO_WATCH = "/home/hrsa/watchfolder"
    w = Watcher(DIRECTORY_TO_WATCH)
    w.run()