#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;

void cloud_cb (boost::shared_ptr<sensor_msgs::PointCloud2> pc2)
{
    sensor_msgs::PointCloud pc;

    if(sensor_msgs::convertPointCloud2ToPointCloud(*pc2, pc)) {
        pub.publish(pc);
    } else {
        ROS_ERROR("Could not convert from PointCloud2 to PointCloud.");
    }
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pointcloud_converter_node");
  ros::NodeHandle nh;

  pub = nh.advertise<sensor_msgs::PointCloud>("pointcloud_out", 100000);

  ros::Subscriber sub = nh.subscribe("pointcloud2_in", 100000, &cloud_cb);

  // Spin
  ros::spin ();
}