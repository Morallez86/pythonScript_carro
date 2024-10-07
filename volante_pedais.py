import pygame
import carla
import time

# Dead zone threshold: any input value smaller than this is treated as zero
DEAD_ZONE = 0.01  # Adjust this value if needed

class CarlaRacingWheelControl:
    def __init__(self):
        # Initialize Pygame and Joystick
        pygame.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        # Connect to CARLA
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.blueprint_library = self.world.get_blueprint_library()
        self.vehicle_bp = self.blueprint_library.filter('vehicle.tesla.model3')[0]

        # Spawn vehicle
        spawn_point = self.world.get_map().get_spawn_points()[0]
        self.vehicle = self.world.spawn_actor(self.vehicle_bp, spawn_point)
        self.control = carla.VehicleControl()

        # Print joystick details for debugging
        print(f"Joystick Name: {self.joystick.get_name()}")
        print(f"Number of Axes: {self.joystick.get_numaxes()}")

    def apply_dead_zone(self, value, threshold=DEAD_ZONE):
        if abs(value) < threshold:
            return 0.0
        return value

    def _parse_vehicle_keys(self, keys, milliseconds):
        pygame.event.pump()  # Update pygame events

        # Axis 0: Steering
        steering = self.joystick.get_axis(0)  # Steering [-1, 1]
        steering = self.apply_dead_zone(steering)

        # Axis 1: Throttle
        throttle = self.joystick.get_axis(1)  # Throttle [0, -1] (inverted axis, adjust if necessary)
        throttle = self.apply_dead_zone(-throttle)  # Invert axis if needed (positive when pressed)

        # Axis 2: Brake
        brake = self.joystick.get_axis(2)  # Brake [0, 1]
        brake = self.apply_dead_zone(brake)

        # Debugging: Print the joystick input values
        print(f"Steering: {steering}, Throttle: {throttle}, Brake: {brake}")

        # Apply to CARLA vehicle control
        self.control.throttle = throttle
        self.control.brake = brake

        # Steering handling
        self.control.steer = round(steering, 1)
        self.control.hand_brake = keys[pygame.K_SPACE]  # Space key for handbrake

        # Debugging: Print vehicle control values
        print(f"Vehicle Control: Throttle: {self.control.throttle}, Brake: {self.control.brake}, Steer: {self.control.steer}")

    def run(self):
        try:
            while True:
                # Capture key events (or joystick events, if you have any button mappings)
                keys = pygame.key.get_pressed()

                # Calculate milliseconds elapsed since last frame
                milliseconds = 1/60 * 1000  # Assuming 60 FPS for this example

                # Parse joystick inputs for vehicle control
                self._parse_vehicle_keys(keys, milliseconds)

                # Apply vehicle control in CARLA
                self.vehicle.apply_control(self.control)

                # Limit loop to 60 FPS
                time.sleep(1/60)

        finally:
            self.vehicle.destroy()
            pygame.quit()

if __name__ == "__main__":
    racing_wheel_control = CarlaRacingWheelControl()
    racing_wheel_control.run()
