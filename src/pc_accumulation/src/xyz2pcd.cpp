#include "ros/ros.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iomanip>
#include <sstream>

std::vector<std::string> split_string(std::string s, char trenner = ' ') {
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

std::vector<double> split_string_to_doubles(std::string s, char trenner = ' ') {
    std::vector<double> result;

    std::vector<std::string> strings = split_string(s, trenner);

    for(int i = 0; i < strings.size(); i++) {
        result.push_back(std::atof(strings[i].c_str()));
    }

    return result;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "xyz2pcd_node");

    ros::NodeHandle pn("~");

    std::string xyz_filename = "", pcd_filename = "";
    pn.getParam("xyz_filename", xyz_filename);
    pn.getParam("pcd_filename", pcd_filename);
    pcd_filename = split_string(pcd_filename, '.')[0];

    if(xyz_filename.empty() || pcd_filename.empty()) {
        ROS_ERROR("specifiy filename of .xyz file and .pcd file, e.g. \n_xyz_filename:=/path_to_file/example.xyz _pcd_filename:=/path_to_file/example.pcd");
        exit(-1);
    }

    std::ifstream file;
    file.open(xyz_filename.c_str());

    if(!file.is_open()) {
        ROS_ERROR("could not open file %s", xyz_filename.c_str());
    }

    std::string line = "";
    std::vector<double> result;
    
    std::getline(file, line);
    result = split_string_to_doubles(line);

    int size = result.size();

    if(size >= 3 && size < 6) {
        ROS_WARN("writing xyz data to pcd file discarding, anything else will be discarded");

        pcl::PointCloud<pcl::PointXYZ>* cloud = new pcl::PointCloud<pcl::PointXYZ>();
        pcl::PointXYZ p;

        int i = 0;
        int d = 10;
        int part = 0;
        do{
            result = split_string_to_doubles(line);

            p.x = result[0];
            p.y = result[1];
            p.z = result[2];

            try {
                cloud->points.push_back(p);
                i++;
            } catch(std::exception e) {
                std::ostringstream ss(pcd_filename, std::ios_base::app);
                ss << "_" << part << ".pcd";   

                ROS_WARN("\n!!!!!!!!\nRAM full, writing cloud to %s \nProcessing will continue after writing!!!!!!!!", ss.str().c_str());
                ROS_INFO("%d points read", int(cloud->points.size()));
                ROS_INFO("writing pcd file");

                cloud->width = cloud->points.size();
                cloud->height = 1;
                cloud->points.resize(cloud->width * cloud->height);
                cloud->is_dense = false;
                pcl::io::savePCDFileASCII(ss.str().c_str(), *cloud);

                ROS_INFO("finished writing pcd file");
                ROS_INFO("continue processing");

                delete cloud;
                cloud = new pcl::PointCloud<pcl::PointXYZ>();

                cloud->points.push_back(p);
                part++;
            }
            

            if(i % d == 0) {
                ROS_INFO("%d points read", i);
                if(i >= d*10) d*=10;
            }

        } while(std::getline(file, line));

        file.close();

        ROS_INFO("%d points read", i);
        ROS_INFO("writing pcd file");

        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->points.resize(cloud->width * cloud->height);
        cloud->is_dense = false;

        pcl::io::savePCDFileASCII(pcd_filename.c_str(), *cloud);

        ROS_INFO("finished writing pcd file");

    } else if(result.size() >= 6) {
        ROS_WARN("writing xyz and rgb data to pcd file, anything else will be discarded");

        pcl::PointCloud<pcl::PointXYZRGB>* cloud = new pcl::PointCloud<pcl::PointXYZRGB>();
        pcl::PointXYZRGB p;

        int i = 0;
        int d = 10;
        int part = 0;
        do{
            result = split_string_to_doubles(line);

            p.x = result[0];
            p.y = result[1];
            p.z = result[2];
            p.r = result[3];
            p.g = result[4];
            p.b = result[5];

            try {
                cloud->points.push_back(p);
            } catch(std::exception e) {
                std::ostringstream ss(pcd_filename, std::ios_base::app);
                ss << "_" << part << ".pcd";   

                ROS_WARN("\n!!!!!!!!\nRAM full, writing cloud to %s \nProcessing will continue after writing!!!!!!!!", ss.str().c_str());
                ROS_INFO("%d points read", int(cloud->points.size()));
                ROS_INFO("writing pcd file");

                cloud->width = cloud->points.size();
                cloud->height = 1;
                cloud->points.resize(cloud->width * cloud->height);
                cloud->is_dense = false;
                pcl::io::savePCDFileASCII(ss.str().c_str(), *cloud);

                ROS_INFO("finished writing pcd file");
                ROS_INFO("continue processing");
                
                delete cloud;
                cloud = new pcl::PointCloud<pcl::PointXYZRGB>();

                cloud->points.push_back(p);
                part++;
            }
            
            i++;

            if(i % d == 0) {
                ROS_INFO("%d points read", i);
                if(i >= d*10) d*=10;
            }

        } while(std::getline(file, line));

        file.close();

        ROS_INFO("%d points read", i);
        ROS_INFO("writing pcd file");

        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->points.resize(cloud->width * cloud->height);
        cloud->is_dense = false;

        pcl::io::savePCDFileASCII(pcd_filename.c_str(), *cloud);

        ROS_INFO("finished writing pcd file");
    } else {
        ROS_ERROR("could not read .xyz file correctly");
    }

    return 0;

}