# Kinova ROS Control

## Setup

    sudo cp `rospack find kinova_driver`/udev/99-jaco-arm.rules /etc/udev/rules.d/
    sudo udevadm control --reload
    ls -l /dev/zaber_vert

## Debug

Run on hardware:

    roslaunch kinova_control kinova_control.launch

Run on simulation:

    roslaunch kinova_control kinova_control_sim.launch
