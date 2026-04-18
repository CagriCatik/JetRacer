#!/bin/bash

map_dir=$(pwd)
map_name="mymap"

ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '$map_dir/$map_name.pbstream', include_unfinished_submaps: true}"
ros2 run cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename="$map_dir/$map_name.pbstream" -map_filestem="$map_dir/$map_name"
