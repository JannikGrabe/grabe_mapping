#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "nav_msgs/Odometry.h"

#include<fstream>


// publishes scan after pose message for that scan was publishes
class ScanPublisher {

private:
  ros::NodeHandle n_;
  ros::NodeHandle n_private;
  ros::Publisher pub_;
  ros::Subscriber sub_;

  std::string path;

public:
  ScanPublisher() {
    pub_ = n_.advertise<sensor_msgs::PointCloud2>("scan", 1000);

    sub_ = n_.subscribe("pose", 1000, &ScanPublisher::callback, this);

    n_private = ros::NodeHandle("~");

    n_private.getParam("path", path);
    
  }

  void callback(const nav_msgs::Odometry::ConstPtr& odom_msg) { 

        std::ostringstream filenameStream(path + "/scan", std::ios_base::app);
        filenameStream << std::setfill('0') << std::setw(3) << odom_msg->header.seq << ".3d";
        std::string filename = filenameStream.str();

        std::ifstream file;
         
        file.open(filename.c_str());

        if(!file.is_open()) {
          ROS_ERROR("could not open scan file %s", filename.c_str());
        }

        sensor_msgs::PointCloud2 msg; 

        std::vector<double> point_data; 
        int point_count = 0; 
        while(!file.eof()) {
            double x, y, z; 
            file >> x >> z >> y;
            point_data.push_back(x / 100); 
            point_data.push_back(y / 100);
            point_data.push_back(z / 100);
            point_count++;
        }

        msg.header.frame_id = "scanner_frame";
        msg.header.seq = odom_msg->header.seq;
        msg.height = 1; 
        msg.width = point_count; 
        msg.is_bigendian = false; 
        msg.is_dense = false;

        // modifier to set fields of pointcloud: 1 field = xyz
        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2FieldsByString(1,"xyz");
        modifier.resize(point_count);

        //iterators to iterate through pointcloud and fill it
        sensor_msgs::PointCloud2Iterator<float> out_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(msg, "z");

        for(int i = 0; i < point_data.size(); i += 3) {
            *out_x = point_data[i];
            *out_y = point_data[i+1];
            *out_z = point_data[i+2];

            ++out_x;
            ++out_y;
            ++out_z;
        }

        msg.header.stamp = ros::Time::now();

        pub_.publish(msg); 

        ROS_INFO("processed scan %d", odom_msg->header.seq);

        file.close(); 
    }
};
    
int main(int argc, char** argv) {
    ros::init(argc, argv, "scan_publisher");

    ScanPublisher pp;

    ros::spin();
}