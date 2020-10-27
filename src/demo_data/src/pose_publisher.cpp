#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include <fstream>
#include <vector>


// publishes pose read from pose files
int main(int argc, char** argv) {

    ros::init(argc, argv, "pose_publisher");

    ros::NodeHandle n;
    ros::NodeHandle n_private("~");

    ros::Publisher pub = n.advertise<nav_msgs::Odometry>("pose", 1000);

    ros::Rate loop_rate(0.5);

    int file_count = 0;
    std::ifstream file; 
    std::string path;
    n_private.getParam("path", path);
    path += "/scan";

    ros::Duration wait(0.5);
    wait.sleep();

    while(ros::ok()) {

        std::ostringstream filenameStream(path, std::ios_base::app);
        filenameStream << std::setfill('0') << std::setw(3) << file_count << ".pose";
        std::string filename = filenameStream.str();

        std::ifstream file; 
        file.open(filename.c_str());

        if(!file.is_open()) {
            ROS_ERROR("could not open pose file %s", filename.c_str());
        }

        double x, y, z;
        double a, b, g;

        file >> x >> z >> y >> a >> g >> b;

        nav_msgs::Odometry pose_msg;
        pose_msg.header.frame_id = "odom"; 
        pose_msg.header.seq = file_count; 
        pose_msg.header.stamp = ros::Time::now();

        pose_msg.child_frame_id = "base_footprint";

        double to_radiant = M_PI / 180;
        pose_msg.pose.pose.position.x = x / 100;
        pose_msg.pose.pose.position.y = y / 100;
        pose_msg.pose.pose.position.z = z / 100;
        pose_msg.pose.pose.orientation.x = a * to_radiant; 
        pose_msg.pose.pose.orientation.y = b * to_radiant;
        pose_msg.pose.pose.orientation.z = g * to_radiant;
        pose_msg.pose.pose.orientation.w = 1;

        pub.publish(pose_msg);

        ros::spinOnce(); 

        ROS_INFO("processed pose %d", file_count);

        loop_rate.sleep();

        file_count++;
        file.close(); 
    }
}