#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"


void pose_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    static tf::TransformBroadcaster br;

    tf::Transform transform; 
    transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    tf::Quaternion q;
    q.setRPY(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));
    
    transform.setOrigin(tf::Vector3(0,0,1));
    q.setRPY(0,0,0);
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "scanner_frame"));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_broadcaster");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("pose", 1000, &pose_callback);

    ros::spin(); 
}