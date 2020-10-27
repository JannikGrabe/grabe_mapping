#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include <fstream>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class Scan_to_file {
private:

    // node handle
    ros::NodeHandle n;
    ros::NodeHandle n_private;

    // subscriber
    message_filters::Subscriber<sensor_msgs::PointCloud2>* scan_sub;
    message_filters::Subscriber<nav_msgs::Odometry>* odom_sub;

    // Synchronizer that puts together messages from all subscribers
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy; // approximate time policy
    message_filters::Synchronizer<MySyncPolicy>* sync;

    // callback for grouped messages
    void callback(const boost::shared_ptr<sensor_msgs::PointCloud2 const>& scan, const boost::shared_ptr<nav_msgs::Odometry const>& odom);
    
    // parameter for file writing
    bool input_is_meter;
    bool input_is_lefthanded;
    std::string output_path;

    // factor for converting unit
    double converting_unit;

public:
    Scan_to_file();

    ~Scan_to_file();
};