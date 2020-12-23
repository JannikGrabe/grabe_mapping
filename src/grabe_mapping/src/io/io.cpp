#include "io/io.h"
#include <fstream>

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

std::vector<double> IO::split_string_to_doubles(std::string s, char trenner) {
    std::vector<double> result;

    std::vector<std::string> strings = IO::split_string(s, trenner);

    for(int i = 0; i < strings.size(); i++) {
        result.push_back(std::atof(strings[i].c_str()));
    }

    return result;
}

void IO::read_pointcloud_from_xyz_file(pcl::PointCloud<pcl::PointXYZ>* cloud, std::string filename) {
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

        p.x = results[0];
        p.y = results[1];
        p.z = results[2];
        
        cloud->points.push_back(p);
    }

    file.close();

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    cloud->is_dense = false;
}

void IO::read_frame_from_file(Eigen::Matrix4d* frame, std::string filename) {
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

    frame = new Eigen::Matrix4d (results.data());
}

void IO::write_pointcloud_to_xyz_file(pcl::PointCloud<pcl::PointXYZ>* cloud, std::string filename) {
    if(filename.size() <= 4 || filename.substr(filename.size() - 4, filename.size() - 1) != ".xyz") {
        throw Bad_file_exception(filename.c_str(), "IO::write_pointcloud_to_xyz_file", ".xyz missing");
    }
    
    std::ofstream file(filename.c_str());

    if(!file.is_open()) {
        throw Bad_file_exception(filename.c_str(), "IO::write_pointcloud_to_xyz_file", "can't open file");
    }

    for(int i = 0; i < cloud->points.size(); i++) {
        file << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z;

        if(i != cloud->points.size() - 1) {
            file << "\n";
        }
    }

    file.close();
}
