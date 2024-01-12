# Gazebo_Essentials


This repo serves as a tutorial for running gazebo simulations with ROS2. At the time of writing this tutorial, I was using ROS2 Iron and Gazebo 11.



# Gazebo Components

# Robot Model Description

Robot models are defined by urdf files or xacro files. Xacro files are (xml macros) used for more efficiently or succinctly describing your robot by making macro blocks out of informaiton that is repeated in several places of the urdf file. The xacro file is then parsed internally by Gazebo to generate the urdf file. URDF files can also be broken down into several hierarchical xacro files each describing different parts of the robot. A main xacro file then references these individual xacro files.

- Xacro/URDF Tags
  - Links
    - Visual
      - Geometry
      - Origin
      - Material
    - Collision
      - Geometry
      - Origin
    - Inertial
      - Mass
      - Origin
      - Inertia Matrix 3x3
  - Joints
    - Origin
    - Name
    - Parent and Child Links
    - Types   
      - Continuous
      - Revolute
      - Prismatic: Sliding or Linear
      - Fixed
    - Axis
    - Limits
      - Upper and Lower position limits rad/meters
      - Velocity m/s or rad/s
      - Effort N or N.m
  - Material : Allows us to give a “name” to a colour once, and then reuse that name in as many links as we like
  - Gazebo : Lets us specify certain parameters that are used in the Gazebo simulation environment
  - Transmission : Provides more detail about how the joints are driven by physical actuators




**Note**
  - URDF values are in metres
  - Reference [3] is a very nicely written reference to understand URDF and xacro files for describing your robot.



# World Model
- xml documents (.world files) that specifies the world layout and populates it with other objects.



**References:**

1) Neobotix website - https://neobotix-docs.de/ros/platform/index.html
2) Neobotix github page - https://github.com/neobotix
3) Articulated Robotics URDF/Xacro Tutorial: https://articulatedrobotics.xyz/ready-for-ros-7-urdf/
4) 
