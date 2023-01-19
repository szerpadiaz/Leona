# A library providing a class that watches a folder and reads new files as soon as they are there 
# Then, continues the program

from face_detector import FaceDetector
import cv2

import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler


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
                time.sleep(5)
        except:
            self.observer.stop()
            print("Error")

        self.observer.join()

# modify result.txt with the answer
def modifyResultFile(DIRECTORY_TO_WATCH, result):
    with open(DIRECTORY_TO_WATCH + "/" + "result.txt", "w") as f:
        f.write(result)



class Handler(FileSystemEventHandler):

    @staticmethod
    def on_any_event(event):
        if event.is_directory:
            return None
            

        elif event.event_type == 'modified':
            # Taken any action here when a file is modified.
            file = event.src_path
            print("Received modified event - %s.", file)
            if file[-4:] == ".png":

                img = cv2.imread(file)
                
                faceDetector = FaceDetector()
                bbox, _ = faceDetector.detect_face(img)


                modifyResultFile(DIRECTORY_TO_WATCH, str(bbox))

if __name__ == '__main__':
    DIRECTORY_TO_WATCH = "watchfolder"
    w = Watcher(DIRECTORY_TO_WATCH)
    w.run()