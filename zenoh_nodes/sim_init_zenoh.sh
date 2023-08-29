#!/bin/bash

#Gazebo and previous configuration to work with
#zenoh (zenoh-flow and zenoh-bridge-dds on top).

#Change the RMW implementation to work with cyclone DDS, needed for zenoh-flow:
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION

#Allow loopback interface to communicate using multicast,
#because some nav2 nodes need it. to work properly:
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
sudo ifconfig lo multicast

#Print the interface configuration to make sure everything is right:
ifconfig

#Start Gazebo, RViz2, and Nav2 ROS2 nodes for multirobots:
ROS_LOCALHOST_ONLY=1 ros2 launch nav2_bringup multi_tb3_simulation_launch.py autostart:=True use_sim_time:=True use_composition:=False verbose:=True
