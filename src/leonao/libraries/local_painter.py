##########################################################################################
################## Leonao Canvas ###########################################
##########################################################################################

import tkinter

class Leonao_canvas():
    def __init__(self):
        self.root = tkinter.Tk()
        self.my_canvas = tkinter.Canvas(self.root, bg="white", height=1200, width=1200)
        self.my_canvas.pack()
        self.PEN_WIDTH = 4

    def set_pen_width(self, w):
        self.PEN_WIDTH = w

    def create_line(self, x1, y1, x2, y2):
        self.my_canvas.create_line(x1, y1, x2, y2, fill="black", width=self.PEN_WIDTH)

    def move_to(self, point):
        pass
        #print("Moving to point ", point, " without drawing anything")

    def register_callback(self, ms, cb, *args):
        self.my_canvas.after(ms, cb, *args)

    def mainloop(self):
        self.root.mainloop()

##########################################################################################
################## Leonao painter class ##################################################
##########################################################################################
import pickle

class Leonao_painter:
    def __init__(self):
        self.canvas = Leonao_canvas()
        self.draw_face_outer_paths = []
        self.draw_face_inner_paths = []
        self.next_path = []
        self.DRAWING_DELAY_ms = 1

    def draw_path(self):
        if(len(self.next_path) > 1):
            start_point = self.next_path.pop(0)
            end_point = self.next_path[0]
            x1, y1 = start_point
            x2, y2 = end_point
            self.canvas.create_line(x1, y1, x2, y2)
            self.canvas.register_callback(self.DRAWING_DELAY_ms, self.draw_path)
        else:
            self.canvas.register_callback(self.DRAWING_DELAY_ms, self.draw_face)

    def draw_face(self):
        if(len(self.draw_face_outer_paths)):
            self.next_path = self.draw_face_outer_paths.pop(0)
            self.canvas.move_to(self.next_path[0])
            self.canvas.register_callback(self.DRAWING_DELAY_ms, self.draw_path) 
        elif(len(self.draw_face_inner_paths)):
            self.next_path = self.draw_face_inner_paths.pop(0)
            self.canvas.move_to(self.next_path[0])
            self.canvas.register_callback(self.DRAWING_DELAY_ms, self.draw_path)
        else:
            print("DONE") 

    def draw(self, face_outer_paths, face_inner_paths):
        self.draw_face_outer_paths = face_outer_paths
        self.draw_face_inner_paths = face_inner_paths
        self.canvas.register_callback(self.DRAWING_DELAY_ms, self.draw_face)
        self.canvas.mainloop()    

from path_generator import *

if __name__=="__main__":

    l_painter = Leonao_painter()

    face_info = {"inner": "../test_imgs/00_sketch_inner.bmp", "outer" :  "../test_imgs/00_sketch_outer.bmp", "top_left_point" : [70, 20], "bottom_right_point" : [420, 485]}
    face_info = {"inner": "../test_imgs/obama_inner.bmp", "outer" :  "../test_imgs/obama_outer.bmp", "top_left_point" : [70, 20], "bottom_right_point" : [420, 485]}
    face_info = {"inner": "../test_imgs/david_3_sketch_inner.bmp", "outer" :  "../test_imgs/david_3_sketch_outer.bmp", "top_left_point" : [70, 20], "bottom_right_point" : [420, 485]}
    face_info = {"inner": "../test_imgs/monalisa_sketch_inner.bmp", "outer" :  "../test_imgs/monalisa_sketch_outer.bmp", "top_left_point" : [100, 2], "bottom_right_point" : [450, 467]}
    face_info = {"inner": "../test_imgs/ingo_sketch_inner.bmp", "outer" :  "../test_imgs/ingo_sketch_outer.bmp", "top_left_point" : [93,30], "bottom_right_point" : [417, 464]}
    face_info = {"inner": "../test_imgs/michael_sketch_inner.bmp", "outer" :  "../test_imgs/michael_sketch_outer.bmp", "top_left_point" : [152,0], "bottom_right_point" : [358, 276]}
    face_info = {"inner": "../test_imgs/ines_sketch_inner.bmp", "outer" :  "../test_imgs/ines_sketch_outer.bmp", "top_left_point" : [0,0], "bottom_right_point" : [512, 512]}

    face_paths_gen = Face_paths_generator(face_info)
    face_outer_paths = face_paths_gen.get_face_outer_path()
    face_inner_paths = face_paths_gen.get_face_inner_path()  

    face_inner_paths_normalised = face_paths_gen.normalize_face_path(face_inner_paths)
    face_outer_paths_normalised = face_paths_gen.normalize_face_path(face_outer_paths)
    face_paths = {"inner" : face_inner_paths_normalised, "outer": face_outer_paths_normalised}
    face_paths_pkl = open("../watchfolder/face_paths.pkl", "wb")
    pickle.dump(face_paths, face_paths_pkl, protocol=2)
    #print(face_paths)

    l_painter.draw(face_outer_paths, face_inner_paths)
