#!/bin/bash

# $1 rosbag_filename
# $2 scan_topic
# $3 odom_topic
# $4 input_is_meter
# $5 input_is_lefthanded
# $6 output_path

echo "----------- SCAN_TO_FILE SETTINGS -----------"
echo "rosbag filename set to: $1"
echo "Remapping scan topic to: $2"
echo "Remapping odom topic to: $3"
echo "Set input_is_meter parameter: $4"
echo "Set input_is_lefthanded parameter: $5"
echo "Set output_path parameter: $6"
echo "---------------------------------------------"

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd ) #path to current directory

roslaunch "$DIR/../launch/scan_to_file.launch" rosbag_filename:="$1" scan_topic:="$2" odom_topic:="$3" input_is_meter:="$4" input_is_lefthanded:="$5" output_path:="$6"