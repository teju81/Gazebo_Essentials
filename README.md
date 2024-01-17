# Gazebo_Essentials


This repo serves as a tutorial for running gazebo simulations with ROS2. At the time of writing this tutorial, I was using ROS2 Iron and Gazebo 11 (Classic).



# Gazebo Components

# Robot Model Description

Robot models are defined by urdf files or xacro files. Xacro files are (xml macros) used for more efficiently or succinctly describing your robot by making macro blocks out of informaiton that is repeated in several places of the urdf file. The xacro file is then parsed internally by Gazebo to generate the urdf file. URDF files can also be broken down into several hierarchical xacro files each describing different parts of the robot. A main xacro file then references these individual xacro files.

URDF Tags
- Links
  - visual
    - geometry - box/cylinder/sphere or a mesh (collada file)
    - origin
    - material
  - collision
    - geometry
    - origin
  - inertial
    - mass
    - origin
    - inertia Matrix 3x3
- joints
  - origin
  - name
  - parent
  - child
  - types   
    - continuous
    - revolute
    - prismatic: Sliding or Linear
    - fixed
  - axis
  - limits
    - upper - position limits rad/meters
    - lower - position limits rad/meters
    - velocity -  m/s or rad/s
    - effort - N or N.m
- material : Allows us to give a “name” to a colour once, and then reuse that name in as many links as we like
- transmission : Provides more detail about how the joints are driven by physical actuators
- gazebo
  - plugins
    - name - Any name of your choice
    - filename (libgazebo_ros_diff_drive.so, libgazebo_ros_joint_state_publisher.so, libgazebo_ros_camera.so, libgazebo_ros_imu_sensor.so, libgazebo_ros_ray_sensor.so)
    - 

- ros2_control
  - diff

Xacro Tags
- Xacro:Args: Referenced by using ${arg <arg_name>}
  - name
  - default
- Xacro:Properties
  - name
  - value
- Xacro:include
  - filename
- Xacro:Macro
  - name
  - parameters - referenced within the macro definition using ${<parameter_name>}
- Xacro:if - xacro statements in between these tags get included when value is true (can be set using parameters or args)
  - value: boolean value

A typival Xacro file will be defined as below, where xacro properties are defined in one file and reused in 

```
<?xml version="1.0"?>
<robot name="volta_robot" xmlns:xacro="https://ros.org/wiki/xacro">
    <!-- xacro includes -->
    <xacro:include filename="$(find volta_description)/urdf/my_robot_properties.xacro"/>
    <xacro:include filename="$(find volta_description)/urdf/components/my_robot_1.urdf.xacro"/>
    <xacro:include filename="$(find volta_description)/urdf/components/my_robot_2.urdf.xacro"/>

   <!-- xacro args -->

   <!-- xacro macro defining: typically defined in one of the include files -->
   <xacro:macro name="my_robot_1_macro1" params="par1 *par2">
    ....
   </xacro:macro>

   <!-- xacro macro referencing-->
   <xacro:my_robot_1_macro1 par1="par1">
   ....
   </xacro:my_robot_1_macro>


</robot>
```



**Note**
  - URDF values are in metres
  - Reference [3] is a very nicely written reference to understand URDF and xacro files for describing your robot.
  - Volta Description package in botsync github page is a good reference design
  - Material in visual for sphere/cylinder/box will only show in RVIZ but not in Gazebo. Need to set this explicitly for gazebo. Not needed if visual is a mesh.



# World Model
- xml documents (.world files) that specifies the world layout and populates it with other objects.


# Teleoperating the Volta in Gazebo Environment

**Create Workspace and Launch Gazebo**
Create a ros2 workspace for the simulation by running the following commands

```
mkdir volta_sim_ws
cd volta_sim_ws
mkdir src
```

Then copy all the necessary files from this repo into the ``src`` folder, and from the root directory of the workspace ``volta_sim_ws`` run the following commands:


```
colcon build
ros2 launch volta_description gazebo_launch.py
```

You should be able to see a Gazebo environment in which the volta is spawned and can be teleoperated using the teleop keyboard node (you need to be on this window for the keyboard commands to be interpreted). Note that the gazebo environment can take a while to spin up the first time (you might have to wait for some time).


**Launch RViz with Config File**
In a new terminal launch rviz with config file ``urdf.rviz`` by running the command:

```
rviz2 -d src/volta_description/rviz_params/urdf.rviz
```

**Launch Slamtoolbox with Config File**
In a new terminal launch the slamtoolbox with config file ``mapper_params_online_async.yaml`` by running the command:

```
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./config/mapper_params_online_async.yaml use_sim_time:=true
```

**Add a Map to RViz**
Open Rviz and click on ``Add`` under the Display window on the left, and add a map. Then change the ``Fixed Frame`` (in the display window on the left under ``Global Options``) from ``odom`` to ``map`` (by using the dropdown menu). This will avoid the map from shifting while you move the robot around the environment. Now using the teleop node drive around the robot in the environment and you should see the map being built.

**Change RViz View**
Change the Rviz view of the map to top-down-ortho by clicking on the righthand side scrollbar which will open up the Views menu. Under ``Type`` choose the ``TopDownOrtho`` from the dropdown list. In the previous view ``orbit`` you can rotate the map in 3D which you will no longer be able to do (only 2D allowed).

**Save Map in RViz**
In RViz click on ``Panel``--->``Add New Panel`` (in the menubar) and select the ``SlamToolBoxPlugin``. The plugin should appear on the bottom left under ``Displays`` window, where you can enter the map name in ``Save Map``  and `` Serialiaze Map`` textboxes. The save map feature is meant for external plugins such as Nav2 to use (will create a .data file), while the serialize map feature is for slamtoolbox to use (will create a .posegraph file).





**References:**

1) Neobotix website - https://neobotix-docs.de/ros/platform/index.html
2) Neobotix github page - https://github.com/neobotix
3) Articulated Robotics URDF/Xacro Tutorial: https://articulatedrobotics.xyz/ready-for-ros-7-urdf/
4) SLAM Toolbox: https://www.youtube.com/watch?v=ZaiA3hWaRzE&t=520s&ab_channel=ArticulatedRobotics
5) Nav2 Package: https://www.youtube.com/watch?v=jkoGkAd0GYk&t=573s&ab_channel=ArticulatedRobotics
