import glob  # Module used for finding file paths that match a pattern.
import os  # Module for interacting with the operating system, e.g., to check the OS type.
import sys  # Module used for system-specific parameters and functions.
import random
import time
import numpy as np
import cv2
import math
from collections import deque
from keras.applications.xception import Xception
from keras.layers import Dense, GlobalAveragePooling2D
from keras.optimizers import Adam
from keras.models import Model

import tensorflow as tf
import keras.backend.tensorflow_backend as backend
from threading import Threading


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
REPLAY_MEMORY_SIZE = 5_000
MIN_REPLAY_MEMORY_SIZE = 1_000
MINIBATCH_SIZE = 16
PREDICTION_BATCH_SIZE = 1
TRAINING_BATCH_SIZE = MINIBATCH_SIZE //4
UPDATE_TARGET_EVERY = 5
MODEL_NAME = "Xception"

MEMORY_FRACTION = 0.8
MIN_REWARD = -200

EPISODES = 100

DISCOUNT = 0.99
epsilon = 1
EPSILON_DECAY = 0.95
MIN_EPSILON = 0.001

AGGREGATE_STATS_EVERY = 10

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
        self.destroy_actors()  # Destroy existing actors before resetting
        print("Resetting environment...")
        self.collision_hist = []

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

        # Destroy all actors that were spawned during the episode
        for actor in self.actor_list:
            if actor.is_alive:  # Check if the actor is still alive
                print(f"Destroying actor {actor.type_id}")
                actor.destroy()

        # Now check for any remaining vehicles in the world and destroy them
        all_actors = self.world.get_actors()
        for actor in all_actors:
            if 'vehicle' in actor.type_id:
                print(f"Destroying lingering vehicle: {actor.type_id}")
                actor.destroy()

        # Clear the actor list after destruction
        self.actor_list.clear()  
        print("All actors destroyed.")

class DQNAgent:
    def __init__(self):
        self.model = self.create_model()
        self.target_model = self.create_model()
        self.target_model.set_weights(self.model.get_weights())

        self.replay_memory = deque(maxlen = REPLAY_MEMORY_SIZE)

        self.tensorboard = ModifiedTensorBoard(log_dir=f"logs/{MODEL_NAME}-{int(time.time())}")
        self.target_update_counter = 0

        self.terminate = False
        self_last_logged_episode = 0
        self.training_initialized = False
    
    def create_model(self):
        base_model = Xception(weights=None, include_top=False, input_shape=(IM_HEIGHT, IM_WIDTH, 3))

        x = base_model.output
        x = GlobalAveragePooling2D()(x)

        predictions = Dense(3, activation="linear")(x)
        model = Model(inputs = base_model.input, outputs=predictions)
        model.compile(loss="mse", optimizer=Adam(lr=0.001, metrics=["accuracy"]))
        return model
    
    def update_replay_memory(self, transition):
        self.replay_memory.append(transition)

    def train(self):
        if len(self.replay_memory) < MIN_REPLAY_MEMORY_SIZE:
            return
        
        minibatch = random.sample(self.replay_memory, MINIBATCH_SIZE)

        current_states = np.array([transition[0] for transition in minibatch])/255
        with self.graph.as_default():
            current_qs_list = self.model.predict(current_states, PREDICTION_BATCH_SIZE)
        
        new_current_states = np.array([transition[3] for transition in minibatch])/255
        with self.graph.as_default():
            future_qs_list = self.target_model.predict(new_current_states, PREDICTION_BATCH_SIZE)

        X = []
        y = []

        for index, (current_state, action, reward, new_state, done) in enumerate(minibatch):
            if not done:
                max_future_q = np.max(future_qs_list[index])
                new_q = reward + DISCOUNT * max_future_q
            else:
                new_q=reward
            
            current_qs = current_qs_list[index]
            current_qs[action] = new_q

            X.append(current_state)
            y.append(current_qs)
        
        log_this_step = False
        if self.tensorboard.step > self.last_logged_episode:
            log_this_step = True
            self.last_log_episode = self.tensorboard.step
        
        with self.graph.as_default():
            self.model.fit(np.array(X)/255, np.array(y), batch_size = TRAINING_BATCH_SIZE, verbose=0, shuffle=False, callbacks=[self.tensorboard] if log_this_step else None)

        if log_this_step:
            self.target_update_counter +=1
        
        if self.target_update_counter > UPDATE_TARGET_EVERY:
            self.target_model.set_weights(self.model.get_weights())
            self.target_update_counter = 0
    
    def get_qs(self, state):
        return self.model.predict(np.array(state).reshape(-1*state.shape)/255)[0]

    def train_in_loop(self):
        X = np.random.uniform(size= (1, IM_HEIGHT, IM_WIDTH, 3)).astype(np.float32)
        y = np.random.uniform(size=(1,3)).astype(np.float32)
        with self.graph.as_default():
            self.model.fit(X,y, verbose=False, btch_size=1)
        
        self.training_initialized = True

        while True:
            if self.terminate:
                return
            self.train()
            time.sleep(0.01)

# MAIN LOOP TO TEST THE ENVIRONMENT
if __name__ == "__main__":
    FPS = 60
    ep_rewards = [-200]

    random.seed(1)
    np.random.seed(1)
    tf.set_random_seed(1)

    gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction = MEMORY_FRACTION)
