import glob # Module used for finding file paths that match a pattern.
import os # Module for interacting with the operating system, e.g., to check the OS type.
import sys # Module used for system-specific parameters and functions.
import random
import time
import numpy as np
import cv2
import math

# Try block to safely add CARLA's Python API to the system path.
try:
    # 'sys.path.append()' adds a new path to Python's list of module search paths.
    # 'glob.glob()' is used to find the exact CARLA .egg file that matches the current Python version and OS.
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,  # Python major version, e.g., 3 for Python 3.x
        sys.version_info.minor,  # Python minor version, e.g., 7 for Python 3.7
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

SHOW_PREVIEW = False
IM_WIDTH = 640
IM_HEIGHT = 480
SECONDS_PER_EPISODE = 10

class CarEnv:
    SHOW_CAM = SHOW_PREVIEW
    STEER_AMT = 1.0
    im_width = IM_WIDTH
    im_height = IM_HEIGHT
    front_camera = None

    def __init__(self):
        # Try to connect to CARLA server
        try:
            print("Connecting to CARLA...")
            self.client = carla.Client("localhost", 2000)
            self.client.set_timeout(10.0)
            print("Connected to CARLA server")
        except Exception as e:
            print(f"Failed to connect to CARLA: {e}")
            return

        self.world = self.client.get_world()
        self.blueprint_library = self.world.get_blueprint_library()
        self.model_3 = self.blueprint_library.filter("model3")[0]
        self.actor_list = []  # Initialize the actor_list here
        print("CarEnv initialized")

    def reset(self):
        self.destroy_actors()
        print("Resetting environment...")
        self.collision_hist = []
        self.actor_list = []  # Reinitialize the list

        # Spawn the vehicle
        self.transform = random.choice(self.world.get_map().get_spawn_points())
        self.vehicle = self.world.spawn_actor(self.model_3, self.transform)
        self.actor_list.append(self.vehicle)
        print("Vehicle spawned")

        # Camera setup
        self.rgb_cam = self.blueprint_library.find('sensor.camera.rgb')
        self.rgb_cam.set_attribute("image_size_x", f"{self.im_width}")
        self.rgb_cam.set_attribute("image_size_y", f"{self.im_height}")
        self.rgb_cam.set_attribute("fov", f"110")

        transform = carla.Transform(carla.Location(x=2.5, z=0.7))
        self.sensor = self.world.spawn_actor(self.rgb_cam, transform, attach_to=self.vehicle)
        self.actor_list.append(self.sensor)
        self.sensor.listen(lambda data: self.process_img(data))
        print("Camera sensor attached")

        # Set the vehicle to stop at the start
        self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0))
        time.sleep(4)

        # Collision sensor setup
        colsensor = self.blueprint_library.find("sensor.other.collision")
        self.colensor = self.world.spawn_actor(colsensor, transform, attach_to=self.vehicle)
        self.actor_list.append(self.colensor)
        self.colensor.listen(lambda event: self.collision_data(event))
        print("Collision sensor attached")

        # Ensure camera is initialized
        while self.front_camera is None:
            time.sleep(0.01)
        
        self.episode_start = time.time()

        self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0))

        print("Environment reset complete")
        return self.front_camera

    def collision_data(self, event):
        print("Collision detected")
        self.collision_hist.append(event)

    def process_img(self, image):
        i = np.array(image.raw_data)
        i2 = i.reshape((self.im_height, self.im_width, 4))
        i3 = i2[:, :, :3]
        if self.SHOW_CAM:
            cv2.imshow("", i3)
            cv2.waitKey(1)
        self.front_camera = i3
        return i3 / 255.0

    def step(self, action):
        if action == 0:
            self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=-1 * self.STEER_AMT))
            print("Steering left")
        elif action == 1:
            self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0))
            print("Going straight")
        elif action == 2:
            self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=1 * self.STEER_AMT))
            print("Steering right")

        # Calculate speed in km/h
        v = self.vehicle.get_velocity()
        kmh = int(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))
        print(f"Speed: {kmh} km/h")

        # Determine if the episode is done and the reward
        if len(self.collision_hist) != 0:
            print("Episode done: collision occurred")
            done = True
            reward = -200
        elif kmh < 50:
            done = False
            reward = -1
        else:
            done = False
            reward = 1

        # End the episode if the time limit is reached
        if self.episode_start + SECONDS_PER_EPISODE < time.time():
            print("Episode done: time limit reached")
            done = True

        return self.front_camera, reward, done, None
    
    def destroy_actors(self):
        print("Destroying all actors...")
        for actor in self.actor_list:
            if actor.is_alive:  # Optional: Check if the actor is still alive
                actor.destroy()
        self.actor_list = []  # Clear the list after destroying actors
        print("All actors destroyed.")


# MAIN LOOP TO TEST THE ENVIRONMENT
if __name__ == "__main__":
    env = CarEnv()
    print("Resetting environment for the first episode...")
    state = env.reset()

    for step_num in range(100):  # Simulate 100 steps, for example
        action = random.choice([0, 1, 2])  # Choose a random action: left, straight, right
        state, reward, done, _ = env.step(action)
        print(f"Step {step_num + 1}, Reward: {reward}")
        
        if done:
            print("Episode finished")
            break
