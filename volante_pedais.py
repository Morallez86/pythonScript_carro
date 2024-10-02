import pygame
import carla
import time

# Initialize Pygame and Joystick
pygame.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Connect to CARLA
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
blueprint_library = world.get_blueprint_library()
vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]

# Spawn vehicle
spawn_point = world.get_map().get_spawn_points()[0]
vehicle = world.spawn_actor(vehicle_bp, spawn_point)
control = carla.VehicleControl()

# Function to normalize the throttle and brake inputs (if needed)
def normalize_pedal_value(value):
    # If your pedals go from 1.0 (unpressed) to 0.0 (fully pressed), this will invert the value.
    return max(0.0, min(1.0, 1.0 - value))

try:
    while True:
        pygame.event.pump()  # Process event queue

        # Get joystick input
        steering = joystick.get_axis(0)  # Axis 0: Steering [-1, 1]
        throttle = joystick.get_axis(1)  # Axis 1: Throttle [0, 1] (may need normalization)
        brake = joystick.get_axis(2)     # Axis 2: Brake [0, 1] (may need normalization)
        gear = joystick.get_axis(3)      # Axis 3: Gear (if you want to use it)

        # Optionally normalize throttle and brake if values are inverted
        throttle = normalize_pedal_value(throttle)  # Normalize if inverted
        brake = normalize_pedal_value(brake)        # Normalize if inverted

        # Set CARLA vehicle controls
        control.steer = steering
        control.throttle = throttle
        control.brake = brake

        # Apply vehicle control in CARLA
        vehicle.apply_control(control)

        # Limit loop to 60 FPS
        time.sleep(1/60)

finally:
    vehicle.destroy()
    pygame.quit()
