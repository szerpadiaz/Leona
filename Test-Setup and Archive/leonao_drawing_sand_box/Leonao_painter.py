from Face_paths_generator import Face_paths_generator
from Leonao_canvas import Leonao_canvas
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

def eliminate_short_simple_paths(paths, min_points):
    total_new_points = 0
    new_paths = []
    for path in paths:
        if len(path) > min_points:
            new_paths.append(path)

    print("TOTAL PATHS = ",  len(new_paths), " ; TOTAL POINTS = ", total_new_points)

    return new_paths

def eliminate_out_of_range(paths, min, max):
    total_new_points = 0
    new_paths = []
    for path in paths:
        new_path = []
        for point in path:
            x_in_range = min[0] <= point[0] <= max[0]
            y_in_range = min[1] <= point[1] <= max[1]
            if x_in_range and y_in_range:
                new_path.append(point)
                total_new_points += 1
            else:
                if len(new_path) > 1:
                    new_paths.append(new_path)
                new_path = []

        if len(new_path) > 1:
            new_paths.append(new_path)

    print("TOTAL PATHS = ",  len(new_paths), " ; TOTAL POINTS = ", total_new_points)

    return new_paths

def normalize_paths(simple_paths, top_left_point, bottom_right_point):
    height = bottom_right_point[1] - top_left_point[1]
    width = bottom_right_point[0] - top_left_point[0]

    x_offset = top_left_point[0]
    y_offset = top_left_point[1]

    new_paths = []
    for subpath in simple_paths:
        new_path = []
        print("Paths points ", len(subpath))
        for x, y in subpath:
            new_x = (x - x_offset) / width
            new_y = (y - y_offset) / height
            new_path.append((new_x, new_y))
        new_paths.append(new_path)
    return new_paths

if __name__=="__main__":

    l_painter = Leonao_painter()

    face_paths_gen = Face_paths_generator()

    image = ['00_sketch_outer.bmp', '00_sketch_inner.bmp']
    image = ['david_3_sketch_outer.bmp', 'david_3_sketch_inner.bmp']
    #image = ['obama_outer.bmp', 'obama_inner.bmp']

    face_outer_paths = face_paths_gen.get_face_outer_path(image[0])
    face_inner_paths = face_paths_gen.get_face_inner_path(image[1])   
    
    top_left_point = [70, 20]
    bottom_right_point = [420,485]
    face_outer_paths = eliminate_out_of_range(face_outer_paths, top_left_point, bottom_right_point)
    face_outer_paths_normalised = normalize_paths(face_outer_paths, top_left_point, bottom_right_point)

    face_inner_paths = eliminate_out_of_range(face_inner_paths, top_left_point, bottom_right_point)
    
    # The simple path could still contain paths with few points
    # we could define a criteria to delete them
    face_inner_paths = eliminate_short_simple_paths(face_inner_paths, 10)
    
    face_inner_paths_normalised = normalize_paths(face_inner_paths, top_left_point, bottom_right_point)
    
    face_paths = {"inner" : face_inner_paths_normalised, "outer": face_outer_paths_normalised}

    face_paths_pkl = open("face_paths_david_4.pkl", "wb")
    pickle.dump(face_paths, face_paths_pkl, protocol=2)
    #print(face_paths)

    l_painter.draw(face_outer_paths, face_inner_paths)