#include <ros/ros.h>
#include <laser_assembler/AssembleScans.h>
#include <platform_laser_rotation/velocity_info.h>
using namespace laser_assembler;

void callback(boost::shared_ptr<platform_laser_rotation::velocity_info> vel) {
  ROS_INFO("velocity: %f", vel->target_velocity);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_assembler_client");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  double rate;
  pnh.param("rate", rate, double(3.0));

  ROS_INFO("rate set to %f", rate);


  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud>("assembled_pointcloud", 5);
  ros::service::waitForService("assemble_scans");
  ros::ServiceClient client = nh.serviceClient<AssembleScans>("assemble_scans");
  AssembleScans srv;


  ros::Time reference = ros::Time::now();

  ros::Time now = ros::Time::now();

  while(nh.ok()) {
    now = ros::Time::now();

    if(now.toSec() - reference.toSec() >= rate) {
      srv.request.begin = reference;
      srv.request.end   = now;

      if (client.call(srv)) {
        pub.publish(srv.response.cloud);
      }
      else {
        ROS_INFO("assemble_scan Service call failed\n");
      }

      reference = ros::Time::now();
    }
  }

  return 0;
}