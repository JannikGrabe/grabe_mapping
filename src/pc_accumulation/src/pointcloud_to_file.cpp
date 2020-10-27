#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <fstream>
#include <tf/transform_listener.h>
#include <math.h>


void cloud_cb(boost::shared_ptr<sensor_msgs::PointCloud2> cloud_msg) {

    static int count = 0;


    // write pointcloud to file
    try {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        if(cloud->height == 0 || cloud->width == 0) {
            ROS_ERROR("%s", "Scan could not be written to file. Scan will be discarded.");
            return;
        }

        std::ostringstream fileNameScanStream("/home/jannik/Bachelorarbeit/data/old_bag/scan", std::ios_base::app);
        fileNameScanStream << std::setfill('0') << std::setw(3) << count << ".3d";
        std::string fileName = fileNameScanStream.str();

        ROS_INFO("%s", fileName.c_str());
        ROS_INFO("height: %d", cloud->height);
        ROS_INFO("width: %d", cloud->width);

        std::ofstream file(fileName.c_str());

        for(int i = 0; i < cloud->size(); i++) {
            file << cloud->points[i].x*100 << " ";
            file << cloud->points[i].z*100 << " ";
            file << cloud->points[i].y*100;

            if(i < cloud->size()-1) {
                file << "\n";
            }
        }

        file.close();
    } catch(std::exception e) {
        ROS_ERROR("%s", e.what());
        return;
    }

    // count will only be incremented if all operations worked. If not all the files written will be overwritten next iteration.
    count++;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_to_file");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("pointcloud", 1000, &cloud_cb);

    ros::spin();
}