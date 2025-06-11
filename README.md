# PLEGSROS
 Robot operating system proof of concept for a pediatric lower limb exoskeleton developed at University of Houston  
 
To Build:  
`cd ~/PLEGS.ROS/ROS2_PLEGS`  
`colcon build --symlink-install`  
`source install/setup.bash`  

To launch the system:  
`ros2 launch robot_control_system robot_control.launch.py use_sim:=true`  

The purpose of use_sim is to be able to test the system from WSL but you can run without it in a fresh Ubuntu install with ROS2 




