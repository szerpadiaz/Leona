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
