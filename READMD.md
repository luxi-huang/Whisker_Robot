# README.md for Whisker Robot Senor Mapping Package
- Author：Luxi Huang
- Skills: SLAM, Mapping, Sensor-data-analysis, ROS, C++
---

##  Package description:
* This SLAM project is using whisker touching sensor to detect object and building a map. 
* It implement [Whisker Physics Simulator](https://github.com/SeNSE-lab/whiskitphysics)simulation to get whisker sensor data.
* Then it transfer the whisker 3D sensor data to 2D laser scan data and implement with  [Slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) to build the map. 
 
 <p align="middle"> <img src="https://github.com/luxi-huang/Whisker_Robot/blob/master/img/Whisker_simulator.gif?raw=true" alt="drawing" /> </p>  


 <p align="middle"> <img src="https://github.com/luxi-huang/Whisker_Robot/blob/master/img/whisker.gif?raw=true" alt="drawing" /> </p>  

## 
 <p align="middle"> <img src="https://github.com/luxi-huang/Whisker_Robot/blob/master/img/Map.png?raw=true" alt="drawing" /> </p>  

## Package files:
### 1. `src/object_detection_node.cpp`:
- This file initial `NodeHandle` and create `object_detection` node. It includes `ObjectDetect` class to convert whisker sensor data to 2D scan data and build the map. 

### 2. `src/object_detect.cpp`:
- This is the Class Constructor for `ObjectDetect`.

### 3. `include/turtle_rect.cpp`:
- Header file for the `ObjectDetect` class.

### 4. `launch/object_detect.launch` :
-  The launch file include all node files for whisker sensor data detect objects and build mapping.  

### 5. data/
- It contains all whisker sensor data which got from [Whisker Physics Simulator](https://github.com/SeNSE-lab/whiskitphysics).

## How to use files
1. Clone the files and using wstool to download all related packages.

2. ``` $ roslaunch whisker object_detect.launch ```

3. ``` $ rostopic echo /scan ```

4. open Rviz, change frame to map, and add map and other topics.