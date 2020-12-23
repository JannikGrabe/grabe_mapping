#include <vector>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "exceptions/bad_file_exception.h"
#include "exceptions/bad_point_exception.h"

class IO {
public:

    static std::vector<std::string> split_string(std::string s, char trenner = ' ');

    static std::vector<double> split_string_to_doubles(std::string s, char trenner = ' ');

    static void read_pointcloud_from_xyz_file(pcl::PointCloud<pcl::PointXYZ>* cloud, std::string filename);

    static void read_frame_from_file(Eigen::Matrix4d* frame, std::string filename);

    static void write_pointcloud_to_xyz_file(pcl::PointCloud<pcl::PointXYZ>* cloud, std::string filename);
};