#include "io/io.h"
#include <fstream>

double IO::scale_factor = 1.0;

std::vector<std::string> IO::split_string(std::string s, char trenner) {
    std::vector<std::string> result;

    std::string buf = "";
    for(int i = 0; i < s.size(); i++) {
        if((s.at(i) == trenner || s.at(i) == '\n')) {
            if(buf.size() > 0) { 
                result.push_back(buf);
                buf = "";
            }
        }
        else {
            buf += s.at(i);
        }

        if(i+1 == s.size() && buf.size() > 0) {
            result.push_back(buf);
        } 
    }

    return result;
}

std::vector<double> IO::split_string_to_doubles(std::string s) {
    std::vector<double> result;

    std::stringstream ss(s.c_str());
    double d = 0;
    while(ss >> d) {
        result.push_back(d);
    }

    return result;
}

void IO::read_pointcloud_from_xyz_file(pcl::PointCloud<pcl::PointXYZ>* &cloud, std::string filename) {
    std::ifstream file;
    file.open(filename.c_str());

    if(!file.is_open()) {
        throw Bad_file_exception(filename.c_str(), "IO::read_pointcloud_from_xyz_file", "can't open file");
    }
    
    cloud = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointXYZ p;

    std::string line = "";
    std::vector<double> results;

    while(std::getline(file, line)) {
        
        results.clear();
        results = IO::split_string_to_doubles(line);

        if(results.size() != 3) {
           throw Bad_point_exception(filename.c_str(), "IO::read_pointcloud_from_xyz_file", "wrong number of doubles");
        }

        p.x = results[0] * scale_factor;
        p.y = results[1] * scale_factor;
        p.z = results[2] * scale_factor;
        
        cloud->points.push_back(p);
    }

    file.close();

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    cloud->is_dense = false;
}

void IO::read_frame_from_file(double* eulerAngles, double* translation, std::string filename) {
    int size = filename.size();
    if(size < 14 
    || filename.substr(filename.size()-14, 4) != "scan" 
    || int(filename.at(size-10)) > 57 || int(filename.at(size-10)) < 48
    || int(filename.at(size-9)) > 57 || int(filename.at(size-9)) < 48
    || int(filename.at(size-8)) > 57 || int(filename.at(size-8)) < 48
    || filename.substr(size-7, 7) != ".frames") {
        throw Bad_file_exception(filename.c_str(), "IO::read_frame_from_file", "should look like this: scanXXX.frames");
    }
    
    std::fstream file(filename.c_str());

    if(!file.is_open()) {
        throw Bad_file_exception(filename.c_str(), "IO::read_frame_from_file", "can't open file");
    }

    std::string line = "";
    std::string last_line = "";

    while(std::getline(file, line)) {
        last_line = line;
    }

    std::vector<double> results = IO::split_string_to_doubles(last_line);

    if(results.size() != 17) {
        throw Bad_point_exception(filename.c_str(), "IO::read_frame_from_file", "wrong number of doubles");
    }

    double* mat = new double[16]();

    for(int i = 0; i < 16; i++) {    
        mat[i] = results[i];
    }

    IO::getEulerFromMatrix4f(mat, eulerAngles, translation);

    double to_rad = M_PI / 180;
    eulerAngles[0] *= to_rad;
    eulerAngles[1] *= to_rad;
    eulerAngles[2] *= to_rad;
    // eulerAngles[0] *= -to_rad;
    // eulerAngles[1] *= -to_rad;
    // eulerAngles[2] *= -to_rad;
    translation[0] *= scale_factor;
    translation[1] *= scale_factor;
    translation[2] *= scale_factor;
}

void IO::read_pose_from_file(double* eulerAngles, double* translation, std::string filename) {
    int size = filename.size();
    if(size < 12 
    || filename.substr(filename.size()-12, 4) != "scan" 
    || int(filename.at(size-8)) > 57 || int(filename.at(size-8)) < 48
    || int(filename.at(size-7)) > 57 || int(filename.at(size-7)) < 48
    || int(filename.at(size-6)) > 57 || int(filename.at(size-6)) < 48
    || filename.substr(size-5, 5) != ".pose") {
        throw Bad_file_exception(filename.c_str(), "IO::read_pose_from_file", "should look like this: scanXXX.pose");
    }
    
    std::fstream file(filename.c_str());

    if(!file.is_open()) {
        throw Bad_file_exception(filename.c_str(), "IO::read_pose_from_file", "can't open file");
    }

    file >> translation[0] >> translation[1] >> translation[2] >> eulerAngles[0] >> eulerAngles[1] >> eulerAngles[2];
    
    double to_rad = M_PI/180;
    eulerAngles[0] *= to_rad;
    eulerAngles[1] *= to_rad;
    eulerAngles[2] *= to_rad;
    translation[0] *= scale_factor;
    translation[1] *= scale_factor;
    translation[2] *= scale_factor;
}

void IO::write_pointcloud_to_xyz_file(pcl::PointCloud<pcl::PointXYZI>* cloud, std::string filename) {
    if(filename.size() <= 4 || filename.substr(filename.size() - 4, filename.size() - 1) != ".xyz") {
        throw Bad_file_exception(filename.c_str(), "IO::write_pointcloud_to_xyz_file", ".xyz missing");
    }
    
    std::ofstream file(filename.c_str());

    if(!file.is_open()) {
        throw Bad_file_exception(filename.c_str(), "IO::write_pointcloud_to_xyz_file", "can't open file");
    }

    for(int i = 0; i < cloud->points.size(); i++) {
        
        file << cloud->points[i].x*scale_factor << " " << cloud->points[i].y*scale_factor << " " << cloud->points[i].z*scale_factor;
        
        if(i != cloud->points.size() - 1) {
            file << "\n";
        }
    }

    file.close();
}

void IO::getEulerFromMatrix4f(const double *matrix4f, double *eulerAngles, double *translation) {
    double _trX, _trY;

    // Calculate Y-axis angle
    if(matrix4f[0] > 0.0) {
        eulerAngles[1] = asin(matrix4f[8]);
    } else {
        eulerAngles[1] = M_PI - asin(matrix4f[8]);
    }

    double  C    =  cos( eulerAngles[1] );
    if ( fabs( C ) > 0.005 )  {                 // Gimbal lock?
        _trX      =  matrix4f[10] / C;             // No, so get X-axis angle
        _trY      =  -matrix4f[9] / C;
        eulerAngles[0]  = atan2( _trY, _trX );
        _trX      =  matrix4f[0] / C;              // Get Z-axis angle
        _trY      = -matrix4f[4] / C;
        eulerAngles[2]  = atan2( _trY, _trX );
    } else {                                    // Gimbal lock has occurred
        eulerAngles[0] = 0.0;                       // Set X-axis angle to zero
        _trX      =  matrix4f[5];  //1                // And calculate Z-axis angle
        _trY      =  matrix4f[1];  //2
        eulerAngles[2]  = atan2( _trY, _trX );
    }

    eulerAngles[0] = eulerAngles[0];
    eulerAngles[1] = eulerAngles[1];
    eulerAngles[2] = eulerAngles[2];

    if (translation != 0) {
        translation[0] = matrix4f[12];
        translation[1] = matrix4f[13];
        translation[2] = matrix4f[14];
    }
}

void IO::getQuaternionFromMatrix4f(const double *matrix4f, double *quaternion, double *translation) {

    double T, S, X, Y, Z, W;

    T = 1 + matrix4f[0] + matrix4f[5] + matrix4f[10];

    if ( T > 0.00000001 ) { // to avoid large distortions!
        S = sqrt(T) * 2;
        X = ( matrix4f[9] - matrix4f[6] ) / S;
        Y = ( matrix4f[2] - matrix4f[8] ) / S;
        Z = ( matrix4f[4] - matrix4f[1] ) / S;
        W = 0.25 * S;
    } else if ( matrix4f[0] > matrix4f[5] && matrix4f[0] > matrix4f[10] )  { // Column 0:
        S  = sqrt( 1.0 + matrix4f[0] - matrix4f[5] - matrix4f[10] ) * 2;
        X = 0.25 * S;
        Y = (matrix4f[4] + matrix4f[1] ) / S;
        Z = (matrix4f[2] + matrix4f[8] ) / S;
        W = (matrix4f[9] - matrix4f[6] ) / S;
    } else if ( matrix4f[5] > matrix4f[10] ) {                    // Column 1:
        S  = sqrt( 1.0 + matrix4f[5] - matrix4f[0] - matrix4f[10] ) * 2;
        X = (matrix4f[4] + matrix4f[1] ) / S;
        Y = 0.25 * S;
        Z = (matrix4f[9] + matrix4f[6] ) / S;
        W = (matrix4f[2] - matrix4f[8] ) / S;
    } else {                                            // Column 2:
        S  = sqrt( 1.0 + matrix4f[10] - matrix4f[0] - matrix4f[5] ) * 2;
        X = (matrix4f[2] + matrix4f[8] ) / S;
        Y = (matrix4f[9] + matrix4f[6] ) / S;
        Z = 0.25 * S;
        W = (matrix4f[4] - matrix4f[1] ) / S;
    }

    quaternion[0] = W;
    quaternion[1] = -X;
    quaternion[2] = -Y;
    quaternion[3] = -Z;

    IO::normalize(quaternion);

    if (translation != 0) {
        translation[0] = matrix4f[12];
        translation[1] = matrix4f[13];
        translation[2] = matrix4f[14];
    }
}

void IO::normalize(double *x, int size)
{
  double sum_squared = 0;
  for(int i = 0; i < size; i++) {
      sum_squared += x[i]*x[i];
  } 
  
  double norm = sqrt(sum_squared);

  x[0] /= norm;
  x[1] /= norm;
  x[2] /= norm;
  x[3] /= norm;
}