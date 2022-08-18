### Example program to save several sensor data including bounding box
### Sensors: RGB Camera (+BoundingBox), Semantic Lidar
### By Mukhlas Adib
### 2020
### Last tested on CARLA 0.9.10.1

### CARLA Simulator is licensed under the terms of the MIT license
### For a copy, see <https://opensource.org/licenses/MIT>
### For more information about CARLA Simulator, visit https://carla.org/

import glob
import os
import sys
import time
from carla import ColorConverter as cc
# try:
#     sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
#         sys.version_info.major,
#         sys.version_info.minor,
#         'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
# except IndexError:
#     print('carla not found')
#     pass

import carla

import argparse
import logging
import random
import queue
import numpy as np
from matplotlib import pyplot as plt
import cv2
import carla_vehicle_annotator as cva

### Set to True if you need to save the data in darknet training format, False otherwise
save_darknet = True
###

# IM_HEIGHT = 1080
# IM_WIDTH = 1920

IM_HEIGHT = 640
IM_WIDTH = 640

def retrieve_data(sensor_queue, frame, timeout=1):
    while True:
        try:
            data = sensor_queue.get(True, timeout)
        except queue.Empty:
            return None
        if data.frame == frame:
            return data


def main(car_model, cam_height, veh_location, veh_orientation):
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=50,
        type=int,
        help='number of vehicles (default: 10)')
    argparser.add_argument(
        '-tm_p', '--tm_port',
        metavar='P',
        default=8000,
        type=int,
        help='port to communicate with TM (default: 8000)')

    args = argparser.parse_args()

    vehicles_list = []
    nonvehicles_list = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    try:
        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(2.0)
        world = client.get_world()

        print('\nRUNNING in synchronous mode\n')
        settings = world.get_settings()
        traffic_manager.set_synchronous_mode(True)
        if not settings.synchronous_mode:
            synchronous_master = True
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)
        else:
            synchronous_master = False

        blueprints = world.get_blueprint_library().filter('vehicle.[fam]*')
        # blueprint2 = world.get_blueprint_library().filter('vehicle.audi*')
        # blueprints = blueprint1.append(blueprint2)
        print(blueprints, type(blueprints))
        # spawn_points = world.get_map().get_spawn_points()
        # number_of_spawn_points = len(spawn_points)
        #
        # if args.number_of_vehicles < number_of_spawn_points:
        #     random.shuffle(spawn_points)
        # elif args.number_of_vehicles > number_of_spawn_points:
        #     msg = 'Requested %d vehicles, but could only find %d spawn points'
        #     logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
        #     args.number_of_vehicles = number_of_spawn_points
        #
        # SpawnActor = carla.command.SpawnActor
        # SetAutopilot = carla.command.SetAutopilot
        # FutureActor = carla.command.FutureActor
        #
        # # --------------
        # # Spawn vehicles
        # # --------------
        # batch = []
        # for n, transform in enumerate(spawn_points):
        #     if n >= args.number_of_vehicles:
        #         break
        #     blueprint = random.choice(blueprints)
        #     if blueprint.has_attribute('color'):
        #         color = random.choice(blueprint.get_attribute('color').recommended_values)
        #         blueprint.set_attribute('color', color)
        #     if blueprint.has_attribute('driver_id'):
        #         driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
        #         blueprint.set_attribute('driver_id', driver_id)
        #     blueprint.set_attribute('role_name', 'autopilot')
        #     batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True)))
        #     spawn_points.pop(0)
        #
        # for response in client.apply_batch_sync(batch, synchronous_master):
        #     if response.error:
        #         logging.error(response.error)
        #     else:
        #         vehicles_list.append(response.actor_id)
        #
        # print('Created %d npc vehicles \n' % len(vehicles_list))

        # -----------------------------
        # Spawn ego vehicle and sensors
        # -----------------------------
        q_list = []
        idx = 0

        # tick_queue = queue.Queue()
        # world.on_tick(tick_queue.put)
        # q_list.append(tick_queue)
        # tick_idx = idx
        # idx = idx + 1
        #
        # # Spawn ego vehicle
        # ego_bp = random.choice(blueprints)
        # ego_transform = random.choice(spawn_points)
        # ego_vehicle = world.spawn_actor(ego_bp, ego_transform)
        # vehicles_list.append(ego_vehicle)
        # ego_vehicle.set_autopilot(True)
        # print('Ego-vehicle ready')

        # Spawn a traffic light
        traffic_light = world.get_actors().filter('traffic.traffic_light*')[2]
        position = traffic_light.get_location()
        new_location = carla.Location(x=8, y=-1, z=cam_height)
        traffic_light_transform = carla.Transform(position + new_location,
                                                  carla.Rotation(yaw=-90))

        tick_queue = queue.Queue()
        world.on_tick(tick_queue.put)
        q_list.append(tick_queue)
        tick_idx = idx
        idx = idx + 1

        # Spawn ego vehicle
        new_location = carla.Location(x=8, y=-1, z=2)
        spawn_point = carla.Transform(position + new_location + carla.Location(y=-veh_location), carla.Rotation(yaw=veh_orientation))
        # ego_bp = client.get_world().get_blueprint_library().filter('vehicle.bmw.grandtourer')[0]
        # ego_bp = client.get_world().get_blueprint_library().filter('ambulance')[0]
        # ego_transform = random.choice(spawn_point)

        # ego_bp = random.choice(client.get_world().get_blueprint_library().filter('vehicle*'))

        vehicles = client.get_world().get_blueprint_library().filter('vehicle.*')
        cars = [x for x in vehicles if int(x.get_attribute('number_of_wheels')) != 2]
        ego_bp = random.choice(cars)
        ego_vehicle = world.spawn_actor(ego_bp, spawn_point)
        vehicles_list.append(ego_vehicle)
        # ego_vehicle.set_autopilot(True)
        print('Ego-vehicle ready')

        # Spawn RGB camera
        cam_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        cam_bp.set_attribute('sensor_tick', '1.0')
        cam_bp.set_attribute('fov', '110')
        cam_bp.set_attribute('image_size_x', str(IM_WIDTH))
        cam_bp.set_attribute('image_size_y', str(IM_HEIGHT))
        # cam = world.spawn_actor(cam_bp, cam_transform, attach_to=ego_vehicle)
        cam = world.spawn_actor(cam_bp, traffic_light_transform)
        nonvehicles_list.append(cam)
        cam_queue = queue.Queue()
        cam.listen(cam_queue.put)
        q_list.append(cam_queue)
        cam_idx = idx
        idx = idx + 1
        print('RGB camera ready')

        # Spawn LIDAR sensor
        lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
        lidar_bp.set_attribute('sensor_tick', '1.0')
        lidar_bp.set_attribute('channels', '64')
        lidar_bp.set_attribute('points_per_second', '1120000')
        lidar_bp.set_attribute('upper_fov', '40')
        lidar_bp.set_attribute('lower_fov', '-40')
        lidar_bp.set_attribute('range', '100')
        lidar_bp.set_attribute('rotation_frequency', '20')
        lidar_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        # lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=ego_vehicle)
        lidar = world.spawn_actor(lidar_bp, traffic_light_transform)
        nonvehicles_list.append(lidar)
        lidar_queue = queue.Queue()
        lidar.listen(lidar_queue.put)
        q_list.append(lidar_queue)
        lidar_idx = idx
        idx = idx + 1
        print('LIDAR ready')

        # Spawn semantic camera
        seman_cam_bp = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
        seman_cam_bp.set_attribute('sensor_tick', '1.0')
        seman_cam_bp.set_attribute('fov', '110')
        seman_cam_bp.set_attribute('image_size_x', str(IM_WIDTH))
        seman_cam_bp.set_attribute('image_size_y', str(IM_HEIGHT))
        seman_cam = world.spawn_actor(seman_cam_bp, traffic_light_transform)
        nonvehicles_list.append(seman_cam)
        seman_cam_queue = queue.Queue()
        seman_cam.listen(seman_cam_queue.put)
        q_list.append(seman_cam_queue)
        seman_cam_idx = idx
        idx = idx + 1
        print('Semantic camera ready')

        # Begin the loop
        time_sim = 0
        while True:
            # Extract the available data
            nowFrame = world.tick()
            if time_sim <= 3:
                time_sim += settings.fixed_delta_seconds
            # Check whether it's time for sensor to capture data
            elif (time_sim > 3) & (time_sim < 4):
                data = [retrieve_data(q, nowFrame) for q in q_list]
                assert all(x.frame == nowFrame for x in data if x is not None)

                # Skip if any sensor data is not available
                if None in data:
                    continue

                vehicles_raw = world.get_actors().filter('vehicle.*')
                snap = data[tick_idx]
                rgb_img = data[cam_idx]
                lidar_img = data[lidar_idx]
                seg_img_old = data[seman_cam_idx]
                seg_img_old.convert(cc.CityScapesPalette)
                i = np.array(seg_img_old.raw_data)
                i2 = i.reshape(
                    (int(seman_cam.attributes['image_size_y']), int(seman_cam.attributes['image_size_x']), 4))
                seg_img = i2[:, :, :3]
                # Attach additional information to the snapshot
                vehicles = cva.snap_processing(vehicles_raw, snap)

                # Calculating visible bounding boxes
                filtered_out, _ = cva.auto_annotate_lidar(vehicles, cam, rgb_img, lidar_img, seg_img, show_img=rgb_img,
                                                          file_name="{}_{}_{}_{}".format(car_model, cam_height, veh_location, veh_orientation),
                                                          json_path='vehicle_class_json_file.txt')

                # Save the results
                cva.save_output(rgb_img, seg_img_old, filtered_out['bbox'], filtered_out['old_bbox'],
                                filtered_out['3d_bbox'], filtered_out['class'], path='yolo_',
                                file_name="{}_{}_{}_{}".format(car_model, cam_height, veh_location, veh_orientation), save_patched=True,
                                out_format='json')

                # Save the results to darknet format
                if save_darknet: cva.save2darknet(filtered_out['bbox'], filtered_out['class'], rgb_img, file_name="{}_{}_{}_{}".format(car_model, cam_height, veh_location, veh_orientation))
                print("wait")
                time_sim = time_sim + 1
            # else:
            # print(time_sim)
            # if time_sim <=1:
            #     time_sim = time_sim + settings.fixed_delta_seconds
            # elif time_sim < 1:
            #     time_sim = time_sim + settings.fixed_delta_seconds
            else:
                break
            # print(time_sim)
            # time_sim = time_sim + settings.fixed_delta_seconds
            print(time_sim)
    finally:
        try:
            if save_darknet: cva.save2darknet(None, None, None, save_train=True)
        except:
            print('No darknet formatted data directory found')
        try:
            cam.stop()
            lidar.stop()
        except:
            print('Sensors has not been initiated')

        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        print('destroying %d nonvehicles' % len(nonvehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in nonvehicles_list])
        time.sleep(0.5)
    # finally:
    #     try:
    #         if save_darknet: cva.save2darknet(None, None, None, save_train=True)
    #     except:
    #         print('No darknet formatted data directory found')
    #     try:
    #         cam.stop()
    #         lidar.stop()
    #     except:
    #         print('Sensors has not been initiated')
    #
    #     settings = world.get_settings()
    #     settings.synchronous_mode = False
    #     settings.fixed_delta_seconds = None
    #     world.apply_settings(settings)
    #
    #     print('\ndestroying %d vehicles' % len(vehicles_list))
    #     client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
    #
    #     print('destroying %d nonvehicles' % len(nonvehicles_list))
    #     client.apply_batch([carla.command.DestroyActor(x) for x in nonvehicles_list])
    #
    #     time.sleep(0.5)

if __name__ == '__main__':
    try:
        for cam_height in [6]:
            for veh_distance in range(11, 30):
                for veh_orientation in range(10, 370, 10):
                    main(car_model="yolo", cam_height=cam_height, veh_location=veh_distance, veh_orientation=veh_orientation)
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
