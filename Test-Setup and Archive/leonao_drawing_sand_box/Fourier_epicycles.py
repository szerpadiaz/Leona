import math
# Fourier Based on: https://editor.p5js.org/codingtrain/sketches/RfrZibjfL

def dft(x):
    N = len(x)
    K = N
    fourier = []
    for k in range(K):
        re = 0
        im = 0
        for n in range(N):
            angle = (2*math.pi * k * n) / N
            re += x[n] * math.cos(angle)
            im -= x[n] * math.sin(angle)
        re = re / N
        im = im / N
        freq = k
        amplitude = math.sqrt(re*re + im*im)
        phase = math.atan2(im, re)
        fourier.append((re, im, freq, amplitude, phase))
    return fourier

def epi_cycles(x, y, rotation, fourier, time):
    for i in range(len(fourier)):
        re, im, freq, radius, phase = fourier[i]
        x += radius * math.cos(freq * time + phase + rotation)
        y += radius * math.sin(freq * time + phase + rotation)
    return (x, y)

def refine_path_using_fourier_epicycles(input_path, x0, y0, x1, y1, AMP_MULTIPLIER, N_SKIP):
    x = []
    y = []
    fourier_x = []
    fourier_y = []
    for i in range(0, len(input_path),N_SKIP):
        x.append(AMP_MULTIPLIER * input_path[i][0])
        y.append(AMP_MULTIPLIER * input_path[i][1])
    fourier_x = dft(x)
    fourier_y = dft(y)
    fourier_x.sort()
    fourier_y.sort()
    ouput_path = []
    time = 0
    dt = 2*math.pi / len(fourier_y)
    while (time <  2*math.pi):
        vx = epi_cycles(x0, y0, 0, fourier_x, time)
        vy = epi_cycles(x1, y1, math.pi / 2, fourier_y, time)
        ouput_path.append((vx[0], vy[1]))
        time += dt
    return ouput_path