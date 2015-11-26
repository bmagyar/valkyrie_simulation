# valkyrie_simulation
This is a ROS package with possibly necessary components to run a simulation of NASA's Valkyrie in Gazebo with ROS Indigo.  
Find the usage section a bit below.  

*This is Major Tom to Ground Control*  
*Val's stepping through the door*  
*And she's floating*  
*in a most peculiar way*  
*And the stars look very different today*  
(a slightly changed section of Space Oddity)

<img align="center" src="https://raw.githubusercontent.com/bmagyar/valkyrie_simulation/master/img/valkyrie_gazebo_space.png" >

## Usage
`git clone https://github.com/bmagyar/val_description && cd val_description && git checkout feature/gazebo_ros_control`  
`git clone https://github.com/bmagyar/valkyrie_simulation`  

To start the simulation  
`roslaunch valkyrie_gazebo valkyrie_gazebo.launch world:=space`  

This loads Valkyrie in space, next to ISS. ForwardJointGroup controllers using the EffortJointInterface from ros_control are started up by default. Joint states and tf are also available. The sensor plugins for laser,cameras, imu and force torques haven't been added yet. 
To check which controllers are running:  
`rosservice call /controller_manager/list_controllers`  
Adding custom controllers using EffortJointInterface is trivial. Sensor data can be made available within ros_control to ensure proper timing between controller iterations and measurements. For walking, this typically consists of imus and force torque sensors.  

