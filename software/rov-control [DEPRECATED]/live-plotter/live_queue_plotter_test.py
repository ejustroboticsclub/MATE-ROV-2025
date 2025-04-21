import live_queue_plotter
import random
import time
import math

fig = live_queue_plotter.initialize_queue(1000, 10)
i = 0

while True:
    new_point = 50*math.sin(2*i/100) + 50
    i+=1
    live_queue_plotter.update(new_point)
    live_queue_plotter.draw()
    print("<<<main thread active>>>")
    
    time.sleep(0.001)
