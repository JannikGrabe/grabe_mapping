#include "ros/ros.h"
#include "tf/transform_broadcaster.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_test_broadcaster");
    
    ros::NodeHandle n;

    tf::TransformBroadcaster br;
  tf::Transform transform;

  while (n.ok()){
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "velodyne"));
  }
}