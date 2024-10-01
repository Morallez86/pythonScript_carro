import carla
import random
import time

def main():
    # Connect to the client and retrieve the world object
    client = carla.Client('localhost', 2000)  # Connects to CARLA at localhost:2000
    client.set_timeout(10.0)  # Set a timeout in seconds to avoid hanging

    try:
        # Get the world (environment)
        world = client.get_world()

        # Get the blueprint library (all available vehicle types)
        blueprint_library = world.get_blueprint_library()

        # Choose a random vehicle blueprint from the library
        vehicle_bp = random.choice(blueprint_library.filter('vehicle'))

        # Find a random spawn point in the world
        spawn_point = random.choice(world.get_map().get_spawn_points())

        # Spawn the vehicle at the chosen spawn point
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        print(f"Spawned a {vehicle_bp.id} at {spawn_point.location}")

        # Apply full throttle to the vehicle (drive straight)
        vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
        print("Vehicle is moving with full throttle")

        # Let the vehicle move for a few seconds
        time.sleep(15)

        # Apply brake after 5 seconds
        vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
        print("Vehicle stopped")

        # Cleanup: Destroy the vehicle after the simulation
        time.sleep(2)
        vehicle.destroy()
        print("Vehicle destroyed, simulation complete.")

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == '__main__':
    main()
