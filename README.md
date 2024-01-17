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

Run the following command

```
ros2 launch volta_description gazebo_launch.py
```


**References:**

1) Neobotix website - https://neobotix-docs.de/ros/platform/index.html
2) Neobotix github page - https://github.com/neobotix
3) Articulated Robotics URDF/Xacro Tutorial: https://articulatedrobotics.xyz/ready-for-ros-7-urdf/
4) 
