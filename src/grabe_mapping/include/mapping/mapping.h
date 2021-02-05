#ifndef GRABE_MAPPING_MAPPING_H
#define GRABE_MAPPING_MAPPING_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

#include "mapping/slam6d.h"

namespace grabe_mapping {
// SLAM Parameters
struct Parameter_set {
    IOType file_format = UOS;
    int start = 0;
    int end = 1;
    int type_ICP = 1;
    double epsilon_ICP = 0.00001;
    int max_it_ICP = 50;
    double max_p2p_dist_ICP = 25.0;
    int type_SLAM = 0;
    double epsilon_SLAM = 0.5;
    int max_it_SLAM = 50;
    double max_p2p_dist_SLAM = 25.0;
    double max_p2p_dist_finalSLAM = -1.0;
    int type_Loop = 0;
    int max_it_Loop = 100;
    double max_p2p_dist_Loop = 700.0;
    double max_dist_Loop = 500.0;
    double max_dist_finalLoop = 500.0;
    int min_overlap_Loop = -1;
    int loopsize = 20;
    int nns_method = 0;
    int bucket_size = 20;
    PairingMode pairing_mode = CLOSEST_POINT;
    double min_dist = -1;
    double max_dist = -1;
    double red_voxel_size = -1.0;
    int octree_red = 0;
    int random_red = -1;
    bool quiet = true;
    bool very_quiet = true;
    bool match_meta = false;
    bool extrapolate_pose = true;
    int anim = -1;
    bool do_icp = true;
};

class Mapping  {

private:

    Parameter_set free;

    IOType file_format = UOS;
    int start = 0;
    int end = 1;
    int type_ICP = 1;
    double epsilon_ICP = 0.00001;
    int max_it_ICP = 50;
    double max_p2p_dist_ICP = 25.0;
    int type_SLAM = 0;
    double epsilon_SLAM = 0.5;
    int max_it_SLAM = 50;
    double max_p2p_dist_SLAM = 25.0;
    double max_p2p_dist_finalSLAM = -1.0;
    int type_Loop = 0;
    int max_it_Loop = 100;
    double max_p2p_dist_Loop = 700.0;
    double max_dist_Loop = 500.0;
    double max_dist_finalLoop = 500.0;
    int min_overlap_Loop = -1;
    int loopsize = 20;
    int nns_method = 0;
    int bucket_size = 20;
    PairingMode pairing_mode = CLOSEST_POINT;
    double min_dist = -1;
    double max_dist = -1;
    double red_voxel_size = -1.0;
    int octree_red = 0;
    int random_red = -1;
    bool quiet = true;
    bool very_quiet = true;
    bool match_meta = false;
    bool extrapolate_pose = true;
    int anim = -1;
    bool do_icp = true;

    icp6Dminimizer* my_icp6Dminimizer = nullptr;
    graphSlam6D *my_graphSlam6D = nullptr;
    icp6D *my_icp = nullptr;
    loopSlam6D *my_loopSlam6D = nullptr;

    // SLAM
    void updateAlgorithms(int start, int end);

    void matchGraph6Dautomatic(double cldist, int loopsize, vector <Scan *> allScans,
                            icp6D *my_icp6D, bool meta_icp, int nns_method,
                            loopSlam6D *my_loopSlam6D, graphSlam6D *my_graphSlam6D,
                            int nrIt, double epsilonSLAM, double mdml, double mdmll, double graphDist,
                            bool &eP, IOType type, int start, int end); 


    std::vector<double> get_point2point_error(int start_index, int end_index);

    std::vector<double> get_plane2plane_error(int start_index, int end_index);

public: 

    Mapping();
    Mapping(Mapping* mapping);

    // control Mapping
    void lock_parameters();
    std::vector<std::string> check_states();

    // SLAM
    int start_slam6d();
    std::vector<double> get_errors();

    /*
    // PointCloud stuff
    void transform_cloud(pcl::PointCloud<pcl::PointXYZI> *in, pcl::PointCloud<pcl::PointXYZI> *out, double *angles, double *translation);
    void calculate_crispnesses(int scan1, int scan2);
    double calculate_crispness(pcl::PointCloud<pcl::PointXYZI> *in);
    */
    void extractPlanes(Scan* scan, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& planes, std::vector<pcl::ModelCoefficients::Ptr>& coefficients);
    
    // getter
    std::string param_to_string();

    int get_start(bool free = false) { 
        if(free) return this->free.start;
        else return this->start; }
    int get_end(bool free = false)  { 
        if(free) return this->free.end;
        else return this->end; }
    Parameter_set get_parameters() {
        return free;
    }

    // setter
        // SLAM parameters
    void set_file_format(IOType fileformat);
    void set_start(int start);
    void set_end(int end);
    void set_do_icp(bool state);
    bool set_ICP_type(int type);
    void set_epsilon_ICP(double eps);
    void set_max_it_ICP(int it);
    void set_max_p2p_dist_ICP(double dist);
    bool set_SLAM_type(int type);
    void set_epsilon_SLAM(double eps);
    void set_max_it_SLAM(int it);
    void set_max_p2p_dist_SLAM(double dist);
    void set_max_p2p_dist_finalSLAM(double dist);
    bool set_Loop_type(int type);
    void set_max_it_Loop(int it);
    void set_max_p2p_dist_Loop(double dist);
    void set_max_dist_Loop(double dist);
    void set_max_dist_finalLoop(double dist);
    void set_Loopsize(int size);
    void set_min_overlap_Loop(int size);
    bool set_nns_method(int method);
    void set_bucket_size(int size);
    bool set_pairing_mode(int mode);
    void set_min_dist(double dist);
    void set_max_dist(double dist);
    void set_red_voxel_size(double size);
    void set_octree_red(int pts_per_voxel);
    void set_random_red(int every_nth_pt);
    void set_quiet(bool quiet);
    void set_match_meta(bool match);
    void set_extrapolate_pose(bool eP);
    void set_anim(int use_every_nth);
};

}

#endif