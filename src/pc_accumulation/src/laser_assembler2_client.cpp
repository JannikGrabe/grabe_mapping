#include <ros/ros.h>
#include <laser_assembler/AssembleScans2.h>
#include <platform_laser_rotation/velocity_info.h>
#include <tf/transform_listener.h>
#include <fstream>
using namespace laser_assembler;

double vel;

void callback(boost::shared_ptr<platform_laser_rotation::velocity_info> vel) {
  ROS_INFO("velocity: %f", vel->target_velocity);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_assembler2_client");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  double rate;
  pnh.param("rate", rate, double(3.0));

  ROS_INFO("rate set to %f", rate);

  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("assembled_pointcloud2", 5);
  ros::service::waitForService("assemble_scans2");
  ros::ServiceClient client = nh.serviceClient<AssembleScans2>("assemble_scans2");
  AssembleScans2 srv;


  ros::Time reference = ros::Time::now();

  ros::Time now = ros::Time::now();

  int count = 0;
  while(nh.ok()) {
    now = ros::Time::now();

    if(now.toSec() - reference.toSec() >= rate) {
      srv.request.begin = reference;
      srv.request.end   = now;

      // get transform for pose
        tf::TransformListener listener;
        tf::StampedTransform transform;

        try {
            ros::Time now = ros::Time::now() + ros::Duration(0.3);
            listener.waitForTransform("/odom", "/base_footprint", now, ros::Duration(1));
            listener.lookupTransform("/odom", "/base_footprint", now, transform);
        } catch(tf::TransformException e){
            ROS_ERROR("%s",e.what());
            ROS_ERROR("%s","Scan will be discarded");
            continue;
        }

        // write pose to file
        try {
            std::ostringstream fileNamePoseStream("/home/jannik/Bachelorarbeit/data/old_bag/scan", std::ios_base::app);
            fileNamePoseStream << std::setfill('0') << std::setw(3) << count << ".pose";
            std::string fileName = fileNamePoseStream.str();

            std::ofstream file(fileName.c_str());
            
            tf::Vector3 translation = transform.getOrigin();
            double roll, pitch, yaw;
            tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
            double to_deg = 180/M_PI;

            file << translation[0]*100 << " " << translation[2]*100 << " " << translation[1]*100 << "\n";
            file << roll*to_deg << " " << yaw*to_deg << " " << pitch*to_deg;

            file.close();
        
        } catch(std::exception e) {
            ROS_ERROR("%s", e.what());
            ROS_ERROR("%s", "Pose could not be written to file. Scan will be discarded.");
            continue;
        }

      if (client.call(srv)) {
        pub.publish(srv.response.cloud);
        count++;
      }
      else {
        
        ROS_INFO("assemble_scans2 Service call failed\n");
      }

      reference = ros::Time::now();
    }
  }

  return 0;
}