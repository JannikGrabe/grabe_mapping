#ifndef GRABE_MAPPING_MAPPING_H
#define GRABE_MAPPING_MAPPING_H

#include <QObject>
#include <QWidget>
#include <QtConcurrent/QtConcurrent>
#include <map>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

#include "mapping/slam6d.h"

namespace grabe_mapping {

class Mapping : public QWidget {
    Q_OBJECT

private:
    // SLAM Parameters
    IOType file_format = UOS;
    int start = 0;
    int end = -1;
    int improve_start = -1;
    int improve_end = -1;
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
    bool scanserver = false;
    int anim = -1;
    QString loopclose_path;
    QString dir_path;

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


    // work
    QFutureWatcher<void> watcher;
    void (Mapping::*next_process)();
    bool cancelled = false;
    std::vector<double> icp_results;

    // control Mapping
    std::vector<std::string> check_states();
    void start_slam6D();
    void improve_slam6D();
    void write_frames_slam6d();
    void finish_mapping();
    void read_results();
    static int run_command(std::string command);

public: 

    Mapping();

    // control Mapping
    void start_mapping();
    void cancel_mapping();
    void showResults();

    // SLAM
    void do_slam6d();
    void improve_slam6d();
    void write_frames();

    // PointCloud stuff
    void transform_cloud(pcl::PointCloud<pcl::PointXYZI> *in, pcl::PointCloud<pcl::PointXYZI> *out, double *angles, double *translation);
    void calculate_crispnesses(int scan1, int scan2);
    double calculate_crispness(pcl::PointCloud<pcl::PointXYZI> *in);
    void segmentPointCloud();

signals:
    void finished_mapping(int exit_code);

public slots:
    void on_process_finished();

public: 
    // getter
    std::string param_to_string();
    std::vector<double> get_icp_results() const;

    // setter
        // SLAM parameters
    void set_start(int start);
    void set_end(int end);
    void set_improve_start(int start);
    void set_improve_end(int end);
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
    void set_scanserver(bool scanserver);
    void set_anim(int use_every_nth);
    void set_dir_path(QString path);
};

}

#endif