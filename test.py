import pygame
import time

pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

print("Joystick name:", joystick.get_name())
print("Number of axes:", joystick.get_numaxes())

while True:
    pygame.event.pump()
    for i in range(joystick.get_numaxes()):
        print(f"Axis {i}: {joystick.get_axis(i):.2f}", end=' | ')
    print()
    time.sleep(0.2)
