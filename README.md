# CARLA-ground-truth

This project aims to collect ground truth data (i.e., 2D or 3D bounding boxes) of object detection using CARLA. The camera is placed on a traffic light to collect images of vehicles at intersections. 

The main.py takes four parameters. folder is the name of the foler to save the outputs, cam_height is the height of the camera, veh_location is the distance from the vehicle to the camera, and veh_orientation is the orientation of the veh. 

# Steps
1. Launch CARLA simulator from command line.
```
cd carla
make launch
```
2. Click the start button of CARLA simulator.
3. Run the main.py file

# References

[CARLA-2DBBox](https://github.com/MukhlasAdib/CARLA-2DBBox)