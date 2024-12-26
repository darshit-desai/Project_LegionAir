# Project LegionAir

## Introduction
This document summarizes the project proposal for the Swarm Robotics course Research project. 
The project aims to emulate a swarm formation of a roman legion where if a single comrade falls out of formation there is a readjustment of the 
whole formation such that the coverage can be maximized.

Swarming behavior using vision is quite common in the animal kingdom, birds flock together while migrating to locations and maintain formation at the same time just based on limited field of view vision sensors. Our motivation is to use a similar limited FOV swarm of drones to track a mother ship drone in the center at the same time maintaining the formation while the the mother ship guides the flock to the designated target. The second stage of the project involves emulating Roman legion tactics of replacing and maintaining coverage of troops. Roman legions functioned using continuous funneling of troops to the front lines so that exhausted troops can rest or recuperate while at the same time fallen soldiers can be replaced. 
Our objective would be to emulate the second formation where one of the drones in the flock looses track or is sent on a scouting mission ahead, 
while the slower moving mother ship and it's flock reorients itself to account for the missing drone.

* The below image shows the stages through which the swarm will execute a coverage compensation maneuver using just visual data of the tracked target
![](assets/repn1.png)

* The below image shows the change in communication graphs of the drones, here nodes means drones:
![](assets/GraphRepn.png)


* The next image shows the final formation:
![](assets/repn2.png)

## Sensors and Hardware

Each of the follower and a leader drone uses Visual Inertial Odometry taken out from the fisheye camera mounted on the drones for its local position estimate. The follower drones track the leader drone using a hires RGB sensor and a Time of Flight sensor which fuse together using late-sensor fusion to get the position of the target.

## Milestones

- [x] Collect data and retrain the Yolov5n model to identify m500 drone
- [x] Move the model to the drone's gpu, test for inference speed
- [x] Collect rosbag while both the follower and leader drones fly for testing sensor fusion
- [x] Write the ros node for sensor fusion and test the FPS with the rosbag
- [x] Convert ROS nodes written in python to C++ for high speed vectorization with Eigen3 library
- [x] Make builds for Arm64 linux computer and test the build run while on the ground
- [ ] Test run of the ROS nodes on follower drones while in flight
- [ ] Benchmark and measure the position estimated by the tracking system of the follower drones
- [ ] Write the ROS node which acts as a position+Velocity controller of the follower drones such that they are able to follow the leader drones around
- [ ] Test flight of the ROS follower node with both the drones flying simultaneously
- [ ] Write automations for simultaneous takeoff, arming, landing and mode switch of leader and follower drone
- [ ] Test automations by flying one drone at a time and then both drones
- [ ] Program software kill switch ROS service in both the leader and follower drones. Test the service out in flight and then make a GUI for doing the same
- [ ] Replicate the above steps for the rest of the drones
- [ ] Add swarm algorithm ROS nodes in the follower drones to maintain formation and execute coverage compensation maneuver
- [ ] Test flight with 2 followers and 1 leader drone
- [ ] Debug problems
- [ ] Test flight with 4 followers and 1 leader drone

## Videos
The following video is of the testing of sensor fusion with detections coming in from the yolo algorithm, The left frame of the video showing the red dots are the Time of flight points reflected from the drone inside the bbox and the right frame is the yolo detection feed

https://github.com/darshit-desai/Project_LegionAir/assets/36150235/982f17dc-1fbe-4672-a3c1-f174a186cdca

## VOXL2 Configuration
[drone](./drone.md)
- Provides detailed configuration instructions for VOXL2 drone setup
- Documents system components, versions, and dependencies
- Includes configuration files and setup commands for:
  - VOXL TFLite Server
  - VOXL Vision Hub
  - VOXL Camera Server
  - Multi-drone namespace configuration
  - Docker container setup
  - ROS2 workspace configuration
  - ROS2-PX4 bridge integration

## Control Vision Update

The project is organized into the following ROS2 packages:

### PX4-ROS2 Communication Bridge
[px4_ros_com](./px4_ros_com/Offboard_Control_Node_Documentation.md)
- Handles communication between PX4 flight controller and ROS2
- Implements offboard control capabilities for autonomous flight
- Manages drone state monitoring and command publishing

### Transform Management
[your_tf_package](./your_tf_package/Static_Transform_Broadcaster_Node_Documentation.md)
- Manages coordinate frame transformations between different reference frames
- Broadcasts static transforms for sensor mounting positions
- Handles dynamic transforms for moving components

### Point Cloud Processing
[your_pointcloud_package](./your_pointcloud_package/Point_Cloud_Transformer_Node_Documentation.md)
- Processes Time of Flight (ToF) sensor data
- Performs point cloud filtering and registration
- Integrates ToF data with other sensor inputs for improved localization

### Drone Detection
[tflite_prop_detection](./tflite_prop_detection/TFLite_Property_Detection_Node_Documentation.md)
- Implements YOLOv5n model for real-time drone detection
- Optimized for embedded GPU execution
- Provides bounding box and confidence scores for detected drones

Each package contains detailed documentation about its specific functionality, setup instructions, and node implementations.



