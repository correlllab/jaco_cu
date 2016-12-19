# Kinova Jaco2 Drivers @ Correll Lab

Uses ros_control to integrate Jaco with MoveIt!

## Run in Simulation

Start ROS:

    roscore &

Simulated controllers using ros_control:

    roslaunch kinova_control kinova_control_sim.launch

Rviz:

    roslaunch jaco_moveit_config moveit_rviz.launch

MoveIt! MoveGroup:

    roslaunch jaco_moveit_config cu_demo.launch
