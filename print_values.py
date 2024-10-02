import pygame

# Initialize pygame
pygame.init()

# Initialize the first joystick (your racing wheel)
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Get the number of axes (usually steering, throttle, brake, clutch, etc.)
num_axes = joystick.get_numaxes()

print(f"Number of axes: {num_axes}")

# Continuously check the values of each axis
while True:
    pygame.event.pump()  # Process event queue
    
    for i in range(num_axes):
        axis_value = joystick.get_axis(i)
        
        # Print only if the axis value has changed significantly
        if abs(axis_value) > 0.01:  # To avoid small floating-point inaccuracies
            print(f"Axis {i}: {axis_value:.2f}")

    pygame.time.wait(500)  # Wait half a second before printing the values again

#axis0 steering
#axis2 throttle
#axis1 brake
#axis3 gear