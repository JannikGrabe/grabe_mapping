#include "scan_to_file/scan_to_file.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include <fstream>

Scan_to_file::Scan_to_file() {

    this->odom_sub = new message_filters::Subscriber<nav_msgs::Odometry>(this->n, "odom", 1);
    this->scan_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(this->n, "scan", 1);

    n_private = ros::NodeHandle("~");

    n_private.getParam("input_is_meter", this->input_is_meter);
    n_private.getParam("input_is_lefthanded", this->input_is_lefthanded);
    n_private.getParam("output_path", this->output_path);

    this->converting_unit = this->input_is_meter ? 100 : 1;

    this->sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *scan_sub, *odom_sub);
    this->sync->registerCallback(&Scan_to_file::callback, this);

    this->count_pub = n.advertise<std_msgs::Int32>("scan_to _file_count", 10);
}

Scan_to_file::~Scan_to_file() {
    delete this->scan_sub;
    delete this->odom_sub;
}

void Scan_to_file::callback(const boost::shared_ptr<sensor_msgs::PointCloud2 const>& scan, 
    const boost::shared_ptr<nav_msgs::Odometry const>& odom) {
        
    static int file_count = 0;

    // get filenames: ../scanXXX.3d, ../scanXXX.pose
    std::ostringstream filenameStream(this->output_path + "/scan", std::ios_base::app);
    filenameStream << std::setfill('0') << std::setw(3) << file_count;
    std::string scan_filename = filenameStream.str() + ".3d";
    std::string pose_filename = filenameStream.str() + ".pose";

    // write Scan to file
    std::ofstream scan_file(scan_filename.c_str());

    boost::shared_ptr<sensor_msgs::PointCloud2> scan_ptr = boost::const_pointer_cast<sensor_msgs::PointCloud2>(scan);

    sensor_msgs::PointCloud2Iterator<float> out_x(*scan_ptr, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(*scan_ptr, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(*scan_ptr, "z");

    for(int i = 0; i < scan->height*scan->width - 1; i++) {
        if(this->input_is_lefthanded)
            scan_file   << *out_x*converting_unit << " "
                        << *out_y*converting_unit << " "
                        << *out_z*converting_unit << "\n";
        else
            scan_file   << *out_x*converting_unit << " "
                        << *out_z*converting_unit << " "
                        << *out_y*converting_unit << "\n";
        
        ++out_x;
        ++out_y;
        ++out_z;
    }

    scan_file.close();

    ROS_INFO("processed scan %d", file_count);

    // write pose to file
    std::ofstream pose_file(pose_filename.c_str());

    double to_degree = 180/M_PI;

    double x = odom->pose.pose.position.x * converting_unit; 
    double y = odom->pose.pose.position.y * converting_unit; 
    double z = odom->pose.pose.position.z * converting_unit; 
    double a = odom->pose.pose.orientation.x * to_degree; 
    double b = odom->pose.pose.orientation.y * to_degree;
    double g = odom->pose.pose.orientation.z * to_degree;

    if(this->input_is_lefthanded) 
        pose_file << x << " " << y << " " << z << "\n" << a << " " << b << " " << g;
    else 
        pose_file << x << " " << z << " " << y << "\n" << a << " " << g << " " << b;

    pose_file.close();

    ROS_INFO("processed pose %d", file_count);

    this->count_pub.publish(file_count);
    
    file_count++;
}