import live_queue_plotter
import random
import time

fig = live_queue_plotter.initialize_queue(100, 10)

while True:
    new_point = random.randint(0, 80)
    live_queue_plotter.update(new_point)
    live_queue_plotter.draw()
    print("<<<main thread active>>>")
    print(live_queue_plotter.history)
    time.sleep(0.1)