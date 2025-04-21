import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, ifft

def initialize_queue(size: int, duration: float):
    global history, length, start, end, history_limit, time_axis, fig, sub_plot, lines, points_added, fft_ready, fft_lines, fft_fig
    plt.close('all')
    length = size
    history = np.ones(size) * -1  
    start = 0
    end = 0
    points_added = 0
    fft_ready = False
    history_limit = duration
    time_axis = np.linspace(0, history_limit, size)  
    plt.ion()  
    fig, sub_plot = plt.subplots()
    lines, = sub_plot.plot(time_axis, history, "o")
    sub_plot.set_title("History Queue")
    sub_plot.set_ylim(-1, 100)
    fft_fig, fft_sub_plot = plt.subplots()
    fft_sub_plot.set_title("FFT of the data")
    fft_lines, = fft_sub_plot.plot(time_axis[1:], history[1:], "o")
    fft_sub_plot.set_ylim(-1, 100)

def fft_update ():
    if points_added >= length:
        new_fft = fft(history)
        new_fft = [abs(i) for i in new_fft]
        new_fft = new_fft[1:]
        max_amp = max(new_fft)
        scale = 100/max_amp
        new_fft = [i*scale for i in new_fft]
        np_fft = np.array(new_fft)
        fft_lines.set_ydata(np_fft)

def update(input_val: float):
    global start, end, history, points_added
    history[end] = input_val  
    end = (end + 1) % length
    points_added += 1
    fft_update()
    if end == start:  
        start = (start + 1) % length

def get_new_ydata():
    new_ydata = np.zeros(length)
    runner = start
    i = 0
    while runner != end:
        new_ydata[i] = history[runner]
        i += 1
        runner = (runner + 1) % length 
    return new_ydata

def draw():
    lines.set_ydata(get_new_ydata())
    fig.canvas.draw_idle()  
    fig.canvas.flush_events()
    fft_fig.canvas.draw_idle()
    fft_fig.canvas.flush_events()
    plt.pause(0.001)


