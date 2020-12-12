# README.md for Whisker Robot Senor Mapping Package
- Author：Luxi Huang
- Skills: SLAM, Mapping, Sensor-data-analysis, ROS, C++
---

##  Package description:
* This SLAM project is using whisker touching sensor to detect and building a map. 
* It implement (link) simulation to get whisker sensor data. As shown figure 1.
* Then it transfer the whisker data to to 2D laser scan data. 
* It implement with (link) SLAM_toolbox to build map. 

##
 <p align="middle"> <img src="https://github.com/luxi-huang/Turtulebot3-Navigation/blob/master/img/rect.gif?raw=true" alt="drawing" /> </p>  

## Package files:
### 1. `src/object_detection_node.cpp`:
- This file initial `NodeHandle` and create `object_detection` node. It includes `TurtleRect` class to make turtle move in  a rectangular trajectory infinitely.
-  It predict turtle position, and draw `rqt_plot` of absolute pose error (x,y,theta).
-  Turtle can be reset to left corner of rectangular by call `traj_reset` service. 

### 2. `src/turtle_rect.cpp`:
- This is the Class Constructor for `TurtleRect`.

### 3. `include/turtle_rect.cpp`:
- Header file for the `TurtleRect` class.

### 4. `launch/trect.launch` :
-  This launch file runs a window to shows a turtle move in  a rectangular trajectory, and draw `rqt_graph` of show turtle pose error. It launch following files:
   - `turtle_rect` node 
   - `turtlelism` node 
   - `turtle_rect.yaml` file 
   - `rqt_plot` node

### 5. `config/turtle_rect.yaml` :
   - It contains following robot path control parameters:
   - `x`: The x coordinate of the lower left corner of a rectangle
   - `y`: The y coordinate of the lower left corner of a rectangle
   - `width`: The width of the rectangle
   - `height`: The height of the rectangle
   - `trans_vel`: The translational velocity of the robot
   - `rot_vel`: The rotational velocity of the robot
   - `frequency`: The frequency of the control loop

## Launch Files
```
$ roslaunch tsim trect.launch --ros-args
```
Optional Arguments:
   - `plot_gui` （default "false"） : set value to true to draw rqt_plot of robot pose error (x, y, theta).
     - `roslaunch tsim trect.launch plot_gui：=true`

## Result rqt_plot for turtle_rect :

The Below chart is the error between predict turtle pose and real turtle pose. 
 <p align="middle"> <img src="https://github.com/luxi-huang/Turtulebot3-Navigation/blob/master/img/turtleRect_errorPlot.png?raw=true" alt="drawing" /> </p>  