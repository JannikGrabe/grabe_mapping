#include "ros/ros.h"
#include "scan_to_file/scan_to_file.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "scan_to_file_node");

    Scan_to_file scan_to_file;

    ros::spin();

    return 0;
}