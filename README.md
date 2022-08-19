# CARLA-ground-truth

This project aims to collect ground truth data (i.e., 2D or 3D bounding boxes) of object detection using CARLA. The camera is placed on a traffic light to collect images of vehicles at intersections. 

The main.py takes four parameters. folder is the name of the folder to save the outputs, the folders for saving data will be automatically created. Cam_height is the height of the camera, veh_location is the distance from the vehicle to the camera, and veh_orientation is the orientation of the veh. 


# Steps
1. Launch CARLA simulator from command line.
```
cd carla
make launch
```
2. Click the start button of CARLA simulator.
3. Run the main.py file

# Examples

The following figure shows an example of the bounding box genearated by the project. The blue box is the 3D bounding box, the green box is the raw 2D bounding box generated based on the 3D bouding box, and the red box is the improved 2D bouding box using the semantic segmentation image.

![](https://github.com/PeiLi-Sandman/CARLA-ground-truth/blob/main/imgs/yolo_6_11_10.png)


# References

[CARLA-2DBBox](https://github.com/MukhlasAdib/CARLA-2DBBox)
