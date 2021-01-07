#include <vector>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "exceptions/bad_file_exception.h"
#include "exceptions/bad_point_exception.h"

class IO {
public:

    static double scale_factor;

    static std::vector<std::string> split_string(std::string s, char trenner = ' ');

    static std::vector<double> split_string_to_doubles(std::string s);

    static void getEulerFromMatrix4f(const double *matrix4f, double *eulerAngles, double *translation = 0);

    static void getQuaternionFromMatrix4f(const double *matrix4f, double *qaternion, double *translation = 0);

    static void normalize(double *x, int size = 4);

    static void read_pointcloud_from_xyz_file(pcl::PointCloud<pcl::PointXYZI>* &cloud, std::string filename);

    static void read_frame_from_file(double* eulerAngles, double* translation, std::string filename);
    
    static void read_pose_from_file(double* eulerAngles, double* translation, std::string filename);

    static void write_pointcloud_to_xyz_file(pcl::PointCloud<pcl::PointXYZI>* cloud, std::string filename);
};