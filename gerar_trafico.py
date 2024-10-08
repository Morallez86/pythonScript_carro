import carla
import time
import logging
from numpy import random

def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        if int_generation in [1, 2, 3]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []

def spawn_traffic(client, world, number_of_vehicles=30, number_of_walkers=10, filterv='vehicle.*', generationv='All',
                  filterw='walker.pedestrian.*', generationw='2', seed=None, tm_port=8000, car_lights_on=False):
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_list = []
    walkers_list = []
    all_id = []
    traffic_manager = client.get_trafficmanager(tm_port)
    traffic_manager.set_global_distance_to_leading_vehicle(2.5)

    if seed is not None:
        traffic_manager.set_random_device_seed(seed)
    random.seed(seed if seed is not None else int(time.time()))

    settings = world.get_settings()
    synchronous_master = False
    if not settings.synchronous_mode:
        synchronous_master = True
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    blueprints = get_actor_blueprints(world, filterv, generationv)
    blueprintsWalkers = get_actor_blueprints(world, filterw, generationw)

    if not blueprints:
        raise ValueError("Couldn't find any vehicles with the specified filters")
    if not blueprintsWalkers:
        raise ValueError("Couldn't find any walkers with the specified filters")

    blueprints = sorted(blueprints, key=lambda bp: bp.id)
    spawn_points = world.get_map().get_spawn_points()

    if number_of_vehicles > len(spawn_points):
        number_of_vehicles = len(spawn_points)

    random.shuffle(spawn_points)

    # Spawn vehicles
    batch = []
    for n, transform in enumerate(spawn_points):
        if n >= number_of_vehicles:
            break
        blueprint = random.choice(blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        blueprint.set_attribute('role_name', 'autopilot')

        batch.append(carla.command.SpawnActor(blueprint, transform)
                     .then(carla.command.SetAutopilot(carla.command.FutureActor, True, traffic_manager.get_port())))

    for response in client.apply_batch_sync(batch, synchronous_master):
        if response.error:
            logging.error(response.error)
        else:
            vehicles_list.append(response.actor_id)

    # Set automatic vehicle lights update if specified
    if car_lights_on:
        all_vehicle_actors = world.get_actors(vehicles_list)
        for actor in all_vehicle_actors:
            traffic_manager.update_vehicle_lights(actor, True)

    # Spawn Walkers
    percentagePedestriansRunning = 0.0
    percentagePedestriansCrossing = 0.0

    spawn_points = []
    for i in range(number_of_walkers):
        loc = world.get_random_location_from_navigation()
        if loc:
            spawn_point = carla.Transform()
            spawn_point.location = loc
            spawn_points.append(spawn_point)

    batch = []
    walker_speed = []
    for spawn_point in spawn_points:
        walker_bp = random.choice(blueprintsWalkers)
        if walker_bp.has_attribute('is_invincible'):
            walker_bp.set_attribute('is_invincible', 'false')
        if walker_bp.has_attribute('speed'):
            walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
        else:
            walker_speed.append(0.0)
        batch.append(carla.command.SpawnActor(walker_bp, spawn_point))

    results = client.apply_batch_sync(batch, True)
    walker_speed2 = []
    for i, result in enumerate(results):
        if result.error:
            logging.error(result.error)
        else:
            walkers_list.append({"id": result.actor_id})
            walker_speed2.append(walker_speed[i])
    walker_speed = walker_speed2

    # Spawn walker controllers
    walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    batch = []
    for walker in walkers_list:
        batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(), walker["id"]))

    results = client.apply_batch_sync(batch, True)
    for i, result in enumerate(results):
        if result.error:
            logging.error(result.error)
        else:
            walkers_list[i]["con"] = result.actor_id

    # Combine the walkers and controllers
    for walker in walkers_list:
        all_id.append(walker["con"])
        all_id.append(walker["id"])
    all_actors = world.get_actors(all_id)

    world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
    for i in range(0, len(all_id), 2):
        all_actors[i].start()
        all_actors[i].go_to_location(world.get_random_location_from_navigation())
        all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

    print(f'Spawned {len(vehicles_list)} vehicles and {len(walkers_list)} walkers.')
    
    return vehicles_list, walkers_list, all_id

def destroy_traffic(client, vehicles_list, all_id):
    world = client.get_world()
    print(f'Destroying {len(vehicles_list)} vehicles.')
    client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

    all_actors = world.get_actors(all_id)
    for i in range(0, len(all_id), 2):
        all_actors[i].stop()

    print(f'Destroying {len(all_id) // 2} walkers.')
    client.apply_batch([carla.command.DestroyActor(x) for x in all_id])

    time.sleep(0.5)
