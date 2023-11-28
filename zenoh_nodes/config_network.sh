#!/bin/bash

# Gazebo and previous configuration to work with zenoh (zenoh-flow and
# zenoh-bridge-dds on top).

# Stop the daemon in case it's running and change the RMW implementation to work
# with cyclone DDS, needed for zenoh-flow:
ros2 daemon stop
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Allow loopback interface to communicate using multicast, because some nav2
# nodes need it. to work properly:
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
sudo ifconfig lo multicast

# Print the interface configuration to make sure everything is right:
ifconfig
echo -e "RMW_IMPLEMENTATION=\033[0;31m$RMW_IMPLEMENTATION\033[0m"

echo -e "To start the simulation execute the following command:"
echo -e "\033[1;36mROS_LOCALHOST_ONLY=1 ros2 launch nav2_bringup_multirobot multi_tb3_simulation_launch.py autostart:=True use_sim_time:=True use_composition:=False verbose:=True\033[0m"

echo -e "\033[1;33mCOMMON ERRORS:\033[0m"
echo -e " - If Gazebo or RViz2 freeze, give any error or they don't work as expected try to kill the processes (Ctrl+C) and run it again."
echo -e ' - If Gazebo gives an error like "\033[0;31maddress already in use\033[0m" when running it again execute "\033[1;33mkillall gzserver\033[0m" before running it again.'
