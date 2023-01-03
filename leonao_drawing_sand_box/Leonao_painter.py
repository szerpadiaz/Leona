from Face_paths_generator import Face_paths_generator
from Leonao_canvas import Leonao_canvas

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


if __name__=="__main__":

    l_painter = Leonao_painter()

    face_paths_gen = Face_paths_generator()   
    face_outer_paths = face_paths_gen.get_face_outer_path('david_3_sketch.bmp')
    face_inner_paths = face_paths_gen.get_face_inner_path('david_3_sketch.bmp')   
    
    l_painter.draw(face_outer_paths, face_inner_paths)