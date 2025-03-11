import numpy as np
import matplotlib.pyplot as plt
import random
import time

def initialize_queue(size: int, duration: float):
    global history, length, start, end, history_limit, time_axis, fig, sub_plot, lines
    length = size
    history = np.ones(size) * -1  
    start = 0
    end = 0
    history_limit = duration
    time_axis = np.linspace(0, history_limit, size)  
    plt.ion()  
    fig, sub_plot = plt.subplots()
    lines, = sub_plot.plot(time_axis, history, "o")
    sub_plot.set_ylim(0, 100)
    return fig

def update(input_val: float):
    global start, end, history
    history[end] = input_val  
    end = (end + 1) % length
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
    print("updated the figure")
    plt.pause(0.001)

def main_loop():
    while True:
        new_point = random.randint(0, 80)
        update(new_point)
        draw()
        print("<<<main thread active>>>")
        time.sleep(0.1)

if __name__ == '__main__':
    fig = initialize_queue(100, 10)
    main_loop()
