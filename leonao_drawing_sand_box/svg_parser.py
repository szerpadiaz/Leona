#!/usr/bin/python3
# requires svg.path, install it like this: pip3 install svg.path

# converts a list of path elements of a SVG file to simple line drawing commands
from svg.path import parse_path
from svg.path.path import Move, Line, Arc, Close, CubicBezier, QuadraticBezier
from xml.dom import minidom

# See SVG docu: https://www.w3.org/TR/SVG/paths.html#PathDataCubicBezierCommands

def get_paths_from_svg_file(svg_file):
    # read the SVG file
    doc = minidom.parse(svg_file)
    path_strings = [path.getAttribute('d') for path
                    in doc.getElementsByTagName('path')]
    doc.unlink()

    # Workaround until qe can get scale and transform from the file:
    if(svg_file == 'david_3_sketch.svg'):
        scale = (0.1, -0.1)
        translate = (0, 512)
    else:
        scale = (1, 1)
        translate = (0, 0)

    # parse into paths array
    paths = []
    num_of_paths = -1
    for path_string in path_strings:
        path = parse_path(path_string)
        for e in path:
            #print(e)
            if isinstance(e, Move):
                num_of_paths += 1
                x0 = translate[0] + (e.end.real * scale[0])
                y0 = translate[1] + (e.end.imag * scale[1])
                paths.append([])
                paths[num_of_paths].append((x0, y0))
                #print(num_of_moves, "Move", e.end.real, e.end.imag)

            elif isinstance(e, Line):
                x0 = translate[0] + (e.start.real * scale[0])
                y0 = translate[1] + (e.start.imag * scale[1]) 
                x1 = translate[0] + (e.end.real * scale[0])
                y1 = translate[1] + (e.end.imag * scale[1])
                #print("(%.2f, %.2f) - (%.2f, %.2f)" % (x0, y0, x1, y1))
                paths[num_of_paths].append((x1, y1))
            elif isinstance(e, CubicBezier):
                x0 = translate[0] + (e.start.real * scale[0])
                y0 = translate[1] + (e.start.imag * scale[1]) 
                x1 = translate[0] + (e.control1.real * scale[0])
                y1 = translate[1] + (e.control1.imag * scale[1])
                x2 = translate[0] + (e.control2.real * scale[0])
                y2 = translate[1] + (e.control2.imag * scale[1])
                x3 = translate[0] + (e.end.real * scale[0])
                y3 = translate[1] + (e.end.imag * scale[1])
                #print("(%.2f, %.2f) - (%.2f, %.2f)" % (x0, y0, x1, y1))
                #print("(%.2f, %.2f) - (%.2f, %.2f)" % (x1, y1, x2, y2))
                #print("(%.2f, %.2f) - (%.2f, %.2f)" % (x2, y2, x3, y3))
                paths[num_of_paths].append((x1, y1))
                paths[num_of_paths].append((x2, y2))
                paths[num_of_paths].append((x3, y3))
            elif isinstance(e, Close):
                pass
            else:
                print("UNKNOW COMMAND: ", e)
                break
    return paths

if __name__=="__main__":
    
    paths = get_paths_from_svg_file('david_3_sketch.svg')
    for i, path in enumerate(paths):
        if(len(path) > 25):
            print("PATH ", i, "(", len(path),") : ", path[0], "-", path[len(path) - 1])
      
