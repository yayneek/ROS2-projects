## 1: 
Selection of appropriate robot model for the project. The model has to have all physical parameters defined, not only visual ones. UR5 with RG2 gripper was chosen. 
## 2: 
Implementation of the robot model to the Gazebo world along with creation of the target object, which is a cuboid.
## 3: 
Rewriting .sdf file to .urdf for rviz and gz_ros2_control. 
## 4: 
Setting up connection between ros2 and gazebo systems using gz_ros2_control. Result of this step was connection between joint_state_publisher_gui and coordinates of the joints in the gazebo world. Now it was possible to create scanners that could define position of the target object. 
## 5: 
Including LiDAR scanners to the world inside Gazebo world. With scanners in the world it was necessary to transfer message type from scanners to location on XY plane. End effect is a node that subscribes to both scanners data and determines location of the center of the cuboid. 
## 6: 
Originally Moveit2 was meant to be used to solve for InverseKinematics. Unfortunately, due to some instabilites and conflicts inside Moveit2 package it was decided to create IK solver and controllers manually. To achieve this general equations of motion need to be defined and used to solve for IK. Solving IK will be achieved using transformation vectors from Rviz2. Then couple of basic poses for manipulator will be defined e.g. 'ready' or 'goal_poistion'. Finally controller will be properly set up using propper kp kd settings. 

