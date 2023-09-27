# `parallel-curves`
[![CI-Noetic](https://github.com/aforechi/parallel_curves/actions/workflows/action-noetic.yaml/badge.svg)](https://github.com/aforechi/parallel_curves/actions/workflows/action-noetic.yaml) [![ros-action-ci](https://github.com/aforechi/parallel_curves/actions/workflows/action-ros-ci.yaml/badge.svg)](https://github.com/aforechi/parallel_curves/actions/workflows/action-ros-ci.yaml)


<p float="left">
  <img src="doc/parallel_curves_husky.gif" width="600" />
</p>

## Parallel Curves Path-Planning

This is a ROS Global (Path) Planner Plugin that implements the Parallel Curves path planning algorithm.

```
cd ~/catkin_ws/src   
git clone https://github.com/aforechi/parallel_curves.git
cd ..
catkin build
catkin install
rospack plugins --attrib=plugin nav_core
```

## Clearpath Husky Robot Simulator

If you decided to test this plugin using Husky Robot Simulator you can install it with the following instructions:

```
$ cd ~/catkin_ws/src   
$ git clone --branch <distro-branch-name> https://github.com/husky/husky.git
$ cd ..
$ catkin build
$ sudo apt-get install ros-<distro-name>-gazebo-ros-pkgs ros-<distro-name>-gazebo-ros-control
$ sudo apt-get install ros-<distro-name>-multimaster-launch
$ sudo apt-get install ros-<distro-name>-lms1xx
$ rosdep install --from-path src --ignore-src  
$ catkin build
$ source devel/setup.bash

```

## Enable the Laser Scan Sensor

The Navigation Stack needs a perception sensor to work and the husky package does not enable it by default in the main distro branches. So you have to enable at least one sensor manually. The robot description has some available sensors and the simpler one is the laser scan LMS1XX. This sensor is enough to test the path planner. Therefore, **set 1** in the parameter laser_enabled as shown below:

```
<!-- File husky.urdf.xacro -->
<!-- Location: ~/catkin_ws/src/husky/husky_description/urdf/husky.urdf.xacro -->
<xacro:arg name="laser_enabled" default="$(optenv HUSKY_LMS1XX_ENABLED 1)" />
```

## Running the Plugin on the Husky Robot using Gazebo Simulator**

In order to load this plugin you have to change the default value of "base_global_planner" to "parallel_curves/ParallelCurvesRos" in the launch file *move_base.launch* as shown below (you should comment or remove the default global planner value "navfn/NavfnROS").

```
<!-- move_base.launch -->
<!-- ~/catkin_ws/src/husky/husky_navigation/launch/move_base.launch -->
<!-- Planners -->
<!-- <arg name="base_global_planner" default="navfn/NavfnROS"/> -->
<arg name="base_global_planner" default="parallel_curves/ParallelCurvesRos"/>
```

In the configuration YAML file of the navigation planners *planner.yaml* you should insert the parameters values of the new planner:

```
<!-- File planner.yaml -->
<!-- ~/catkin_ws/src/husky/husky_navigation/config/planner.yaml -->
ParallelCurvesRos:
  cell_size: 0.2
  min_dist_btw_nodes: 1.0
  max_iter_inside_circle: 200
  max_range: 10.0

```
**4. Testing the planner with Gazebo Simulator and Rviz**

In three separate terminals, execute these three launch commands:

```
roslaunch husky_gazebo husky_playpen.launch
roslaunch husky_navigation amcl_demo.launch
roslaunch husky_viz view_robot.launch
```
The amcl_demo.launch file launchs the *move_base.launch* file edited above and the all navigation packages. The *view_robot.launch* file run the RViz with the proper topics configuration. 

Lastly, on the left side at "Display" part, change the name of the topic related to Global Planner to visualize it. If you using the DWA Local Planner the topic name is "/move_base/DWAPlannerROS/global_plan".

To test the plugin just click on "2D Nav Goal" button (at the top) and choose a goal location. You can now see that path generated by this planner (in green) and the robot moving to its goal if everything is fine.

## Unit Tests
Launch the tests verification at ```~catkin_ws/build```:

```$ catkin test parallel_curves```

Or run manually with:

```rostest parallel_curves test.launch```

## VSCode Setup

If you want to setup vscode please refer to https://samarth-robo.github.io/blog/2020/12/03/vscode_ros.html


## Errors and Issues

If you find some error or issue, please create a new issue and help me to improve this package.

## Citation

If you use the navigation algorithm from this repository, please cite this work in your papers!

 - A. Baumgarten, L. Resendo, A. Forechi. [**Parallel Curves Path Planning Based on Tangent Segments to Concentric Curves**](https://arxiv.org/abs/). The 6th Iberian Robotics Conference (ROBOT2023), 2023.
 
 ```bibtex
 @InProceedings{macenski2020marathon2,
   title = {Parallel Curves Path Planning Based on Tangent Segments to Concentric Curves},
   author = {Baumgarten, Alysson and Resendo, Leandro and Forechi, Avelino},
   year = {2023},
   booktitle = {The 6th Iberian Robotics Conference (ROBOT2023)},
   url = {https://github.com/aforechi/parallel_curves},
   pdf = {https://arxiv.org/abs/}
 }
```
