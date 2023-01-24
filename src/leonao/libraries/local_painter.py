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
        print("Moving to point ", point, " without drawing anything")

    def register_callback(self, ms, cb, *args):
        self.my_canvas.after(ms, cb, *args)

    def mainloop(self):
        self.root.mainloop()

##########################################################################################
################## Leonao painter class ##################################################
##########################################################################################



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
            self.canvas.PEN_WIDTH = 8
            self.next_path = self.draw_face_outer_paths.pop(0)
            self.canvas.move_to(self.next_path[0])
            self.canvas.register_callback(self.DRAWING_DELAY_ms, self.draw_path) 
        elif(len(self.draw_face_inner_paths)):
            self.canvas.PEN_WIDTH = 3
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