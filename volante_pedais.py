import pygame
import carla
import time
import numpy as np  # Make sure to import numpy
import gerar_trafico as gf

# Dead zone threshold: any input value smaller than this is treated as zero
DEAD_ZONE = 0.01  # Adjust this value if needed

class CarlaRacingWheelControl:
    def __init__(self):
        # Initialize Pygame and Joystick
        pygame.init()

        # Check for available joysticks
        if pygame.joystick.get_count() == 0:
            print("No joystick detected. Please connect a racing wheel or joystick.")
            pygame.quit()
            return

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        # Pygame window for rendering the camera feed
        self.display = pygame.display.set_mode((800, 600), pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.display.set_caption("CARLA Camera View")

        # Connect to CARLA
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.blueprint_library = self.world.get_blueprint_library()
        self.vehicle_bp = self.blueprint_library.filter('vehicle.tesla.model3')[0]

        # Spawn vehicle
        spawn_points = self.world.get_map().get_spawn_points()
        self.vehicle = None
        for spawn_point in spawn_points:
            try:
                self.vehicle = self.world.spawn_actor(self.vehicle_bp, spawn_point)
                break
            except RuntimeError as e:
                print(f"Failed to spawn at {spawn_point.location}, retrying...")

        if not self.vehicle:
            raise RuntimeError("Failed to spawn the vehicle due to collision or invalid points.")

        self.control = carla.VehicleControl()

        # Attach a camera to the vehicle
        camera_bp = self.blueprint_library.find('sensor.camera.rgb')
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))  # Adjust as needed
        self.camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.vehicle)

        # Listen to camera data
        self.camera.listen(self.process_camera_image)

        # Store the latest image for display
        self.latest_image = None

        # State variable to track whether the vehicle is in reverse
        self.is_reverse = False

         # Initialize time of the last reverse toggle
        self.last_toggle_time = 0  # Store the last time reverse was toggled

        self.vehicles_list, self.walkers_list, self.all_id = gf.spawn_traffic(self.client, self.world)

    def apply_dead_zone(self, value, threshold=DEAD_ZONE):
        if abs(value) < threshold:
            return 0.0
        return value

    def process_camera_image(self, image):
        # Convert CARLA image to a format suitable for Pygame
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))  # BGRA format
        array = array[:, :, :3]  # Take only RGB channels
        self.latest_image = array

    def render_camera_feed(self):
        if self.latest_image is not None:
            # Convert the latest camera image to a Pygame surface and display it
            surface = pygame.surfarray.make_surface(self.latest_image.swapaxes(0, 1))
            self.display.blit(surface, (0, 0))
            pygame.display.flip()

    def _parse_vehicle_keys(self, keys, milliseconds):
        pygame.event.pump()  # Update pygame events

        # Axis 0: Steering
        steering = self.joystick.get_axis(0)  # Steering [-1, 1]
        steering = self.apply_dead_zone(steering)

        # Axis 1: Throttle (adjusted for correct range)
        throttle = self.joystick.get_axis(2)  # Throttle [0, -1] (inverted axis, adjust if necessary)
        throttle = self.apply_dead_zone(-throttle)  # Adjust if axis is inverted, positive when pressed
        throttle = max(0.0, throttle)  # Ensure throttle is between 0 and 1

        # Axis 2: Brake
        brake = self.joystick.get_axis(1)  # Brake [0, 1] (possibly inverted axis)
        brake = self.apply_dead_zone(brake)
        brake = max(0.0, brake)  # Ensure brake is between 0 and 1

        # Last axis for reverse
        reverse_axis = self.joystick.get_axis(3)  # Get last axis
        reverse_axis = self.apply_dead_zone(reverse_axis)  # Apply dead zone

        # Invert brake if necessary (if brake reads near 1.0 when not pressed)
        if brake > 0.9:
            brake = 0.0  # No brake when not pressed
        else:
            brake = 1.0 - brake  # Invert brake axis

        # Time-based debouncing for reverse gear
        current_time = time.time()  # Get the current time in seconds
        if reverse_axis < -DEAD_ZONE and current_time - self.last_toggle_time > 0.5:
            # Toggle reverse mode and update last toggle time
            self.is_reverse = not self.is_reverse
            self.last_toggle_time = current_time  # Update the last toggle time

        # Apply to CARLA vehicle control
        self.control.throttle = throttle
        self.control.brake = brake

        # Gear management
        if self.is_reverse:  # If reverse mode is active
            self.control.reverse = True
            self.control.gear = -1  # Set to reverse gear
        elif throttle > 0.0:  # Normal throttle operation
            self.control.reverse = False  # Disable reverse
            self.control.gear = 1  # Set to drive mode (forward gear)
        elif brake > 0.0:  # If brake is applied
            self.control.gear = 0  # Neutral gear when brake is fully applied
        else:
            self.control.gear = 1  # Default to drive mode if no throttle or brake is applied


        # Steering handling
        self.control.steer = round(steering, 1)
        self.control.hand_brake = keys[pygame.K_SPACE]  # Space key for handbrake

    def run(self):
        try:
            if not hasattr(self, 'joystick'):
                print("Joystick not initialized. Exiting...")
                return
            
            while True:
                # Capture key events (or joystick events, if you have any button mappings)
                keys = pygame.key.get_pressed()

                # Calculate milliseconds elapsed since last frame
                milliseconds = 1/60 * 1000  # Assuming 60 FPS for this example

                # Parse joystick inputs for vehicle control
                self._parse_vehicle_keys(keys, milliseconds)

                # Apply vehicle control in CARLA
                self.vehicle.apply_control(self.control)

                # Render the camera feed in the Pygame window
                self.render_camera_feed()

                self.world.tick()

                # Limit loop to 60 FPS
                time.sleep(1/60)

        finally:
            # Clean up on exit
            gf.destroy_traffic(self.client, self.vehicles_list, self.all_id)
            self.vehicle.destroy()
            self.camera.destroy()
            pygame.quit()

if __name__ == "__main__":
    racing_wheel_control = CarlaRacingWheelControl()
    if hasattr(racing_wheel_control, 'joystick'):
        racing_wheel_control.run()
