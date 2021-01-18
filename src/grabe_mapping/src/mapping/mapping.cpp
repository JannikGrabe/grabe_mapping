#include "mapping/mapping.h"
#include "ros/ros.h"
#include "ros/package.h"
#include <QProcess>
#include <fstream>
#include <pcl/common/transforms.h>
#include "io/io.h"
#include <string>
#include <cmath>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Eigenvalues>
#include <pcl/filters/voxel_grid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

namespace grabe_mapping {

Mapping::Mapping() {

    this->slam6d = new My_slam6d();

    this->init_states();  

    QObject::connect(&this->watcher, &QFutureWatcher<void>::finished, this, &Mapping::on_process_finished);
}

// control Mapping
void Mapping::start_mapping() {
    std::vector<std::string> errors = this->check_states();
    if(errors.size() > 0) {

        for(int i = 0; i < errors.size(); i++) {
            std::cout << errors[i] << std::endl;
        }

        emit this->finished_mapping(1);
        return;
    }

    if(this->use_rosbag) 
        this->start_scan_to_file();
    else {
        this->start_slam6D();
    }
}

void Mapping::start_scan_to_file() {
    // build command to run script to launch scan_to_file_node
    std::string scan_to_file_launch_sh = this->script_path + "/scan_to_file_launch.sh ";
    scan_to_file_launch_sh += this->rosbag_filename.toStdString();                                            // rosbag_filename
    scan_to_file_launch_sh += " " + this->scan_topic.toStdString();                                           // scan_topic name
    scan_to_file_launch_sh += " " + this->odom_topic.toStdString();                                           // odom_topic name
    scan_to_file_launch_sh += " " + (this->input_is_meter ? std::string("true") : std::string("false"));      // input_is_meter flag
    scan_to_file_launch_sh += " " + (this->input_is_lefthanded ? std::string("true") : std::string("false")); // input_is_lefthanded flags
    scan_to_file_launch_sh += " " + this->dir_path.toStdString();                                      // path where to put any output files
  
    // open thread for scan_to_file_node
    QFuture<int> scan_to_file_future = QtConcurrent::run(Mapping::run_command, scan_to_file_launch_sh);

    // set watcher to currently last process
    this->watcher.setFuture(scan_to_file_future);
    this->watcher.setObjectName("scan_to_file");

    this->next_process = &Mapping::finish_scan_to_file;
}

void Mapping::finish_scan_to_file() {
    emit this->finished_rosbag();
    this->next_process = &Mapping::start_slam6D;
    this->on_process_finished();
}

void Mapping::start_slam6D() {
    //QFuture<int> slam6D_future = QtConcurrent::run(Mapping::run_command, slam6D);
    QFuture<void> slam6D_future = QtConcurrent::run(this, &Mapping::do_slam6d);

    this->watcher.setFuture(slam6D_future);

    this->next_process = &Mapping::finish_slam6D;
}

void Mapping::improve_slam6D() {
    QFuture<void> slam6D_future = QtConcurrent::run(this, &Mapping::improve_slam6d);

    this->watcher.setFuture(slam6D_future);

    this->next_process = &Mapping::finish_slam6D;
}

void Mapping::finish_slam6D() {
    QFuture<void> slam6D_future = QtConcurrent::run(this, &Mapping::write_frames);

    this->watcher.setFuture(slam6D_future);

    this->next_process = &Mapping::finish_mapping;
}

void Mapping::showResults() {

    // build command to run show
    std::ostringstream oss(ros::package::getPath("grabe_mapping") + "/bin/show ", std::ios_base::app);
    oss << "-s " << this->start << " -e " << this->end << " " << this->dir_path.toStdString();

    std::cout << oss.str() << std::endl;

    QFuture<int> show_future = QtConcurrent::run(Mapping::run_command, oss.str());

    this->watcher.setFuture(show_future);

    // this->next_process = &Mapping::finish_mapping;
}

void Mapping::finish_mapping() {
    if(this->cancelled) {
        emit this->finished_mapping(1);
        this->cancelled = false;
    } else {

        if(export_points) {
            std::string move_export = "mv " + QDir::current().absolutePath().toStdString() + "/points.pts "
                                    + this->export_path.toStdString();
            
            this->run_command(move_export);
        }

        this->read_results();
        emit this->finished_mapping(0);
    }
}

void Mapping::read_results() { 
    this->icp_results.clear();

    for(int i = 1; i < Scan::allScans.size(); i++) {
        unsigned int* np;
        this->icp_results.push_back(this->my_icp->Point_Point_Error(Scan::allScans[i-1], Scan::allScans[i], 
                                                                    this->max_p2p_dist_ICP, np));
    }
}

int Mapping::run_command(std::string command) {
    return system(command.c_str());
}

void Mapping::cancel_mapping() {
    this->cancelled = true;
    this->next_process = &Mapping::finish_mapping;
    if(QString::compare(this->watcher.objectName(), "scan_to_file") == 0)
        this->run_command("rosnode kill /player");
}

// SLAM
void Mapping::updateAlgorithms(int start, int end) {

    // set searchtree and reduction parameter for all scans
    for(int i = start; i <= end; i++) {
        Scan* scan = Scan::allScans[i];

        scan->setRangeFilter(this->max_dist, this->min_dist);
        
        unsigned int types = 0;
        if ((pairing_mode == CLOSEST_POINT_ALONG_NORMAL_SIMPLE) ||
            (pairing_mode == CLOSEST_PLANE_SIMPLE)) {

            types = PointType::USE_NORMAL;
        }

        scan->setReductionParameter(this->red_voxel_size, this->octree_red, PointType(types));
        scan->setSearchTreeParameter(this->nns_method, this->bucket_size);
    }

    /* set icp minimization method to dual quaternion based
    * constructor param: quiet mode?
    */
    if(this->my_icp6Dminimizer != nullptr) delete this->my_icp6Dminimizer;
    switch(this->type_ICP) {
    case 1:
        this->my_icp6Dminimizer = new icp6D_QUAT(quiet);
        break;
    case 2:
        this->my_icp6Dminimizer = new icp6D_SVD(quiet);
        break;
    case 6:
        this->my_icp6Dminimizer = new icp6D_APX(quiet);
        break;
   }
    
    /* set slam method
    * param1: icp minimizer 
    * param2: max p2p dist ICP
    * param3: max p2p dist SLAM
    * param4: max iterations ICP
    * param5: quiet mode?
    * param6: match against meta scan?
    * param7: randomized point reduction (every n-th point or -1 for disabled))
    * param8: extrapolate pose?
    * param9: use only every n-th point for animation or -1 for disabled
    * param10: epsilon ICP
    * param11: nns_method (0 = simple ks, 1 = cached kd)
    * param12: epsilon SLAM
    */
    if(this->my_graphSlam6D != nullptr) delete this->my_graphSlam6D;
    switch(this->type_SLAM) {
    case 0:
        this->my_graphSlam6D = nullptr;
        break;
    case 1:
        this->my_graphSlam6D = new lum6DEuler(
            my_icp6Dminimizer, max_p2p_dist_ICP, max_p2p_dist_SLAM, 
            max_it_ICP, quiet, match_meta, 
            random_red, extrapolate_pose, anim,
            epsilon_ICP, nns_method, epsilon_SLAM
        );
        break;
    case 2:
        this->my_graphSlam6D = new lum6DQuat(
            my_icp6Dminimizer, max_p2p_dist_ICP, max_p2p_dist_SLAM, 
            max_it_ICP, quiet, match_meta, 
            random_red, extrapolate_pose, anim,
            epsilon_ICP, nns_method, epsilon_SLAM
        );
        break;
    case 3:
        this->my_graphSlam6D = new ghelix6DQ2(
            my_icp6Dminimizer, max_p2p_dist_ICP, max_p2p_dist_SLAM, 
            max_it_ICP, quiet, match_meta, 
            random_red, extrapolate_pose, anim,
            epsilon_ICP, nns_method, epsilon_SLAM
        );
        break;
    case 4:
        this->my_graphSlam6D = new gapx6D(
            my_icp6Dminimizer, max_p2p_dist_ICP, max_p2p_dist_SLAM, 
            max_it_ICP, quiet, match_meta, 
            random_red, extrapolate_pose, anim,
            epsilon_ICP, nns_method, epsilon_SLAM
        );
        break;
    }

    /* create icp
    * param1: icp minimizer 
    * param2: max p2p dist ICP
    * param3: max iterations ICP
    * param4: quiet mode?
    * param5: match against meta scan?
    * param6: randomized point reduction (every n-th point or -1 for disabled)
    * param7: extrapolate pose?
    * param8: use only every n-th point for animation or -1
    * param8: epsilon ICP
    * param10: nns_method (0 = simple ks, 1 = cached kd)
    */
    if(this->my_icp != nullptr) delete this->my_icp; 
    this->my_icp = new icp6D(my_icp6Dminimizer,
        max_p2p_dist_ICP, 
        max_it_ICP,
        quiet,
        match_meta, 
        random_red, 
        extrapolate_pose,
        anim, 
        epsilon_ICP, 
        nns_method
    );

    /* create loop closure
    * param1: very quiet mode?
    * param2: icp minimizer
    * param3: max p2p distance loop closing
    * param4: max loop closing iterations
    * param5: random point reduction (every n-th point or -1 for disabled)
    */
    if(this->my_loopSlam6D != nullptr) delete this->my_loopSlam6D;
    switch(this->type_Loop) {
    case 0:
        this->my_loopSlam6D = nullptr;
        break;
    case 2:
        this->my_loopSlam6D = new elch6Dquat(
            very_quiet, 
            my_icp6Dminimizer, 
            max_p2p_dist_Loop, 
            max_it_Loop, 
            random_red, 
            extrapolate_pose, 
            10, // anim = 10 
            epsilon_ICP, 
            nns_method
        );
        break;
    case 3:
        this->my_loopSlam6D = new elch6DunitQuat(
            very_quiet, 
            my_icp6Dminimizer, 
            max_p2p_dist_Loop, 
            max_it_Loop, 
            random_red, 
            extrapolate_pose, 
            10, // anim = 10 
            epsilon_ICP, 
            nns_method
        );
        break;
    case 4:
        this->my_loopSlam6D = new elch6Dslerp(
            very_quiet, 
            my_icp6Dminimizer, 
            max_p2p_dist_Loop, 
            max_it_Loop, 
            random_red, 
            extrapolate_pose, 
            10, // anim = 10 
            epsilon_ICP, 
            nns_method
        );
        break;
    }
}

void Mapping::do_slam6d()
{
    Scan::closeDirectory();
    /* read scans from directory
    * param1: scanserver active?
    * param2: path
    * param3: scan file format
    * param4: first scan
    * param5: last scan
    */ 
    Scan::openDirectory(scanserver, dir_path.toStdString(), file_format, start, end); 
    
    // after open directory all scans will be in static variable Scan::allScans
    if(Scan::allScans.size() == 0) {
        cerr << "No scans found. Did you use the correct format?" << endl;
        return;
    }

    this->updateAlgorithms(this->start, this->end);

    /* do 6DSLAM
    * param1: max distance for loop closing
    * param2: loop size
    * param3: scans
    * param4: icp
    * param5: match against meta?
    * param6: nns method (0 = simple kd, 1 = cached kd)
    * param7: loop algo
    * param8: slam algo
    * param9: max iterations slam
    * param10: epsilon SLAM
    * param11: max p2p dist SLAM
    * param12: max p2p dist final SLAM
    * param13: max dist for final loop closing
    * param14: extrapolate pose?
    * param15: scan file format
    */ 
    this->matchGraph6Dautomatic(
        max_dist_Loop, 
        loopsize, 
        Scan::allScans, 
        my_icp, 
        match_meta, 
        nns_method, 
        my_loopSlam6D, 
        my_graphSlam6D, 
        max_it_SLAM, 
        epsilon_SLAM, 
        max_p2p_dist_SLAM, 
        max_p2p_dist_finalSLAM, 
        max_dist_finalLoop, 
        extrapolate_pose,
        file_format,
        start,
        end
    );
}

void Mapping::improve_slam6d() {
    
    this->max_it_ICP = 50;

    this->improve_start = 20;
    this->improve_end = 40;

    this->updateAlgorithms(this->improve_start, this->improve_end);

    std::vector<Scan*> scans_for_improving;
    int start_index = start - this->improve_start;
    int end_index = end - this->improve_start;

    // remember pose of end node
    double old_end[16]; 
    memcpy(old_end, Scan::allScans[end_index]->get_transMat(), sizeof(old_end)); 

    if(this->my_icp != nullptr) {
            // move scans back to origin 
        for(int i = start_index; i <= end_index; i++) {
            Scan* scan = Scan::allScans[i];
            double trans[16];
            M4inv(scan->getDAlign(), trans);
            scan->transform(trans, Scan::ICP);
            scans_for_improving.push_back(scan);
        }
    }
   
    // merge first scan position with position of scan before
    if(start > this->improve_start) {
        Scan::allScans[start_index]->mergeCoordinatesWithRoboterPosition(Scan::allScans[start_index-1]);
    }
 
    this->matchGraph6Dautomatic(
        max_dist_Loop, 
        loopsize, 
        scans_for_improving, 
        my_icp, 
        match_meta, 
        nns_method, 
        my_loopSlam6D, 
        my_graphSlam6D, 
        max_it_SLAM, 
        epsilon_SLAM, 
        max_p2p_dist_SLAM, 
        max_p2p_dist_finalSLAM, 
        max_dist_finalLoop, 
        extrapolate_pose,
        file_format,
        improve_start,
        improve_end
    );

    // update transmat of any scans that come after the section to improve
    if(end == this->improve_end) {
        return;
    }
    

    double temp[16];
    M4inv(old_end, temp);
    double delta[16];
    MMult(Scan::allScans[end_index]->get_transMat(), temp, delta);
    
    for(int i = end_index + 1; i < Scan::allScans.size(); i++) {
        Scan::allScans[i]->transform(delta, Scan::ICP);
    }
}

void Mapping::write_frames() {
    
    double id[16];
    M4identity(id);

    for(size_t i= 0; i < Scan::allScans.size(); i++) {
        Scan::allScans[i]->clearFrames();
    }

    for(size_t i= 0; i < Scan::allScans.size(); i++) {
        Scan::allScans[i]->transform(id, Scan::ICP, 0);
    }

    // write frames to file:
    const double* p;
    ofstream redptsout(this->loopclose_path.toStdString());
    for(ScanVector::iterator it = Scan::allScans.begin();
        it != Scan::allScans.end();
        ++it)
    {
        Scan* scan = *it;
        p = scan->get_rPos();
        Point x(p[0], p[1], p[2]);
        redptsout << x << endl;
        scan->saveFrames(false);
    }
    redptsout.close();
}

/**
 * This function is does all the matching stuff
 * it iterates over all scans using the algorithm objects to calculate new poses
 * objects could be NULL if algorithm should not be used
 *
 * @param cldist maximal distance for closing loops
 * @param loopsize minimal loop size
 * @param allScans Contains all laser scans
 * @param my_icp6D the ICP implementation
 * @param meta_icp math ICP against a metascan
 * @param nns_method Indicates the nearest neigbor search method to be used
 * @param my_loopSlam6D used loopoptimizer
 * @param my_graphSlam6D used global optimization
 * @param nrIt The number of iterations the global SLAM-algorithm will run
 * @param epsilonSLAM epsilon for global SLAM iteration
 * @param mdml maximal distance match for global SLAM
 * @param mdmll max distance match for global SLAM after all scans are matched
 */
void Mapping::matchGraph6Dautomatic(double cldist, int loopsize, vector <Scan *> allScans,
                            icp6D *my_icp6D, bool meta_icp, int nns_method,
                            loopSlam6D *my_loopSlam6D, graphSlam6D *my_graphSlam6D,
                            int nrIt, double epsilonSLAM, double mdml, double mdmll, double graphDist,
                            bool &eP, IOType type, int start, int end)
{

    double cldist2 = sqr(cldist);

    // list of scan for metascan
    vector < Scan* > metas;

    // graph for loop optimization
    graph_t g;

    int n = allScans.size();

    int loop_detection = 0;
    double dist, min_dist = -1;
    int first = 0, last = 0;

    for(int i = 1 ; i < n ; i++) {
        cout << start + i << "/" << end << endl;

        add_edge(i-1, i, g);

        if(eP && my_icp6D != NULL) {
            allScans[i]->mergeCoordinatesWithRoboterPosition(allScans[i-1]);
        }

        //Hack to get all icp transformations into the .frames Files
        if(i == n-1 && my_icp6D != NULL && my_icp6D->get_anim() == -2) {
            my_icp6D->set_anim(-1);
        }

        /*if(i == 85 || i == 321 || i == 533) {
        my_icp6D->set_anim(1);
        }*/

        if(my_icp6D != NULL){
        cout << "ICP" << endl;
        // Matching strongly linked scans with ICPs
        if(meta_icp) {
            metas.push_back(allScans[i - 1]);
            MetaScan* meta_scan = new MetaScan(metas);
            my_icp6D->match(meta_scan, allScans[i]);
            delete meta_scan;
        } else {
            switch(type) {
            case UOS_MAP:
            case UOS_MAP_FRAMES:
            my_icp6D->match(allScans[0], allScans[i]);
            break;
            case RTS_MAP:
            //untested (and could not work)
            //if(i < 220-22 && i > 250-22) match(allScans[0], CurrentScan);
            my_icp6D->match(allScans[0], allScans[i]);
            break;
            default:
            my_icp6D->match(allScans[i - 1], allScans[i]);
            break;
            }
        }
        } else {
            double id[16];
            M4identity(id);
            allScans[i]->transform(id, Scan::ICP, 0);
        }

        /*if(i == 85 || i == 321 || i == 533) {
        my_icp6D->set_anim(-2);
        }*/

        if(loop_detection == 1) {
            loop_detection = 2;
        }

        for(int j = 0; j < i - loopsize; j++) {
            dist = Dist2(allScans[j]->get_rPos(), allScans[i]->get_rPos());
            if(dist < cldist2) {
                loop_detection = 1;
                if(min_dist < 0 || dist < min_dist) {
                min_dist = dist;
                first = j;
                last = i;
                }
            }
        }

        if(loop_detection == 2) {
            loop_detection = 0;
            min_dist = -1;

            if(my_loopSlam6D != NULL) {
                cout << "Loop close: " << first << " " << last << endl;
                my_loopSlam6D->close_loop(allScans, first, last, g);
                add_edge(first, last, g);
            }

            if(my_graphSlam6D != NULL && mdml > 0) {
                int j = 0;
                double ret;
                do {
                    // recalculate graph
                    Graph *gr = new Graph(i + 1, cldist2, loopsize);
                    cout << "Global: " << j << endl;
                    ret = my_graphSlam6D->doGraphSlam6D(*gr, allScans, 1);
                    delete gr;
                    j++;
                } while (j < nrIt && ret > epsilonSLAM);
            }
        }
    }

    if(loop_detection == 1 && my_loopSlam6D != NULL) {
        cout << "Loop close: " << first << " " << last << endl;
        my_loopSlam6D->close_loop(allScans, first, last, g);
        add_edge(first, last, g);
    }

    

    if(my_graphSlam6D != NULL && mdml > 0.0) {
        int j = 0;
        double ret;
        do {
            // recalculate graph
            Graph *gr = new Graph(n, cldist2, loopsize);
            cout << "Global: " << j << endl;
            ret = my_graphSlam6D->doGraphSlam6D(*gr, allScans, 1);
            delete gr;
            j++;
        } while (j < nrIt && ret > epsilonSLAM);
    }

    if(my_graphSlam6D != NULL && mdmll > 0.0) {
        my_graphSlam6D->set_mdmll(mdmll);
        int j = 0;
        double ret;
        do {
            // recalculate graph
            Graph *gr = new Graph(n, sqr(graphDist), loopsize);
            cout << "Global: " << j << endl;
            ret = my_graphSlam6D->doGraphSlam6D(*gr, allScans, 1);
            delete gr;
            j++;
        } while (j < nrIt && ret > epsilonSLAM);
    }
}


// PointCloud stuff
void Mapping::transform_cloud(pcl::PointCloud<pcl::PointXYZI> *in, pcl::PointCloud<pcl::PointXYZI> *out, double *angles, double *translation) {
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << translation[0], translation[1], translation[2];
    transform.rotate(Eigen::AngleAxisf (angles[0], Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf (angles[1], Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf (angles[2], Eigen::Vector3f::UnitZ()));

    pcl::transformPointCloud(*in, *out, transform);
}

void Mapping::calculate_crispnesses(int scan1, int scan2) {
    std::ostringstream scan1_base(this->dir_path.toStdString() + "/scan", std::ios_base::app);
    scan1_base << std::setfill('0') << std::setw(3) << scan1;

    std::ostringstream scan2_base(this->dir_path.toStdString() + "/scan", std::ios_base::app);
    scan2_base << std::setfill('0') << std::setw(3) << scan2;

    pcl::PointCloud<pcl::PointXYZI>* cloud_pose_transformed = new pcl::PointCloud<pcl::PointXYZI>;
    pcl::PointCloud<pcl::PointXYZI>* cloud_transformed = new pcl::PointCloud<pcl::PointXYZI>;

    IO::scale_factor = 0.01;

    try {

        // CLOUD 1
        // read cloud1
        pcl::PointCloud<pcl::PointXYZI>* cloud1;
        IO::read_pointcloud_from_xyz_file(cloud1, scan1_base.str() + ".3d");

        std::cout << "-> " << this->calculate_crispness(cloud1) << std::endl;

        double * euler = new double[3]();
        euler[0] = 2 * M_PI / 180;
        euler[1] = 0;
        euler[2] = 0;

        double * trans = new double[3]();
        trans[0] = 0.1;
        trans[0] = 0.1;
        trans[0] = 0.1;

        pcl::PointCloud<pcl::PointXYZI>* cloud2 = new pcl::PointCloud<pcl::PointXYZI>();        
        this->transform_cloud(cloud1, cloud2, euler, trans);

        *cloud2 += *cloud1;

        // IO::write_pointcloud_to_xyz_file(cloud1, "/home/jannik/Bachelorarbeit/data/cloud1.xyz");

        // // read pose1 and transform
        // double *euler = new double[3](), *trans = new double[3]();
        // IO::read_pose_from_file(euler, trans, scan1_base.str() + ".pose");
        // this->transform_cloud(cloud1, cloud1, euler, trans);

        // *cloud_pose_transformed += *cloud1;

        // // read frame1 and transform
        // IO::read_frame_from_file(euler, trans, scan1_base.str() + ".frames");
        // this->transform_cloud(cloud1, cloud1, euler, trans);

        // // CLOUD 2
        // // read cloud2
        // pcl::PointCloud<pcl::PointXYZI>* cloud2;
        // IO::read_pointcloud_from_xyz_file(cloud2, scan2_base.str() + ".3d");
        // IO::write_pointcloud_to_xyz_file(cloud2, "/home/jannik/Bachelorarbeit/data/cloud2.xyz");

        // // read pose2 and transform
        // IO::read_pose_from_file(euler, trans, scan2_base.str() + ".pose");
        // this->transform_cloud(cloud2, cloud2, euler, trans);

        // *cloud_pose_transformed += *cloud2;
        // IO::write_pointcloud_to_xyz_file(cloud_pose_transformed, "/home/jannik/Bachelorarbeit/data/cloud_pose_transformed.xyz");

        // // read frame2 and transform
        // IO::read_frame_from_file(euler, trans, scan2_base.str() + ".frames");
        // this->transform_cloud(cloud2, cloud2, euler, trans);

        // // concat cloud1 and cloud2
        // *cloud_transformed = *cloud1 + *cloud2;
        // IO::write_pointcloud_to_xyz_file(cloud_transformed, "/home/jannik/Bachelorarbeit/data/cloud_transformed.xyz");

        std::cout << "-> " << this->calculate_crispness(cloud2) << std::endl;

    } catch (Bad_file_exception e) {
        std::cout << e.what() << std::endl;
        std::cout << e.get_filename() << std::endl;
        std::cout << e.get_source() << std::endl;   
        return;
    } catch (Bad_point_exception e) {
        std::cout << e.what() << std::endl;
        std::cout << e.get_filename() << std::endl;
        std::cout << e.get_source() << std::endl;
        return;
    } catch (std::exception e) {
        std::cout << e.what() << std::endl;
    }
    return;
}

double Mapping::calculate_crispness(pcl::PointCloud<pcl::PointXYZI> *in) {
    
    int N = in->points.size();
    double dev = 1;
    Eigen::Matrix3f cov = Eigen::Matrix3f::Identity();
    double dev2 = dev * dev;
    cov *= dev2;

    // // calculate squared slength of all points
    // for(int i = 0; i < N; i++) {
    //     pcl::PointXYZI* p = &in->points[i];
    //     p->intensity = p->x * p->x + p->y * p->y + p->z * p->z;
    // }

    // // sort points by length
    // std::sort(in->points.begin(), in->points.end(), [](pcl::PointXYZI& p1, pcl::PointXYZI& p2) {
    //     return p1.intensity > p2.intensity;
    // });

    // create kd-tree of point cloud
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(in->makeShared());

    // determine radius for kdtree radius search
    Eigen::EigenSolver<Eigen::Matrix3f> solver(cov, true);
    double max_eigen_val = solver.eigenvalues()[0].real();
    
    double k = 10;
    
    double radius = 2*k*max_eigen_val*dev*dev;
    std::cout << radius << std::endl;

    // caclulate gaussian only of points later in the list 
    std::vector<int> indeces;
    std::vector<float> distances;
    int count = 0;
    double E = 0.0;
    double factor = 1.0 / ( std::pow(2 * M_PI, 3 / 2.0) * cov.determinant() );

    pcl::PointXYZI* pj;
    double pix, piy, piz;
    double px, py, pz;

    for(int i = 0; i < N; i++) {
        if(i % 1000 == 0) std::cout << i << std::endl;

        pix = in->points[i].x;
        piy = in->points[i].y;
        piz = in->points[i].z;

        if(kdtree.radiusSearch(in->points[i], radius, indeces, distances) > 0) {
            for(int j = 0; j < indeces.size(); j++) {
                if(indeces[j] > i) {
                    pj = &in->points[indeces[j]];
                    
                    px = pix - pj->x;
                    py = piy - pj->y;
                    pz = piz - pj->z;
                    
                    E += factor * std::exp(-0.5*px*px*dev2+py*py*dev2+pz*pz*dev2); 
                    count ++;
                }
            }
        }

    }

    E = E / count;
    return E = -std::log(E);
}

void Mapping::segmentPointCloud() {
    std::ostringstream scan1_base(this->dir_path.toStdString() + "/scan", std::ios_base::app);
    scan1_base << std::setfill('0') << std::setw(3) << 4;

    //IO::scale_factor = 0.01;
    IO::scale_factor = 100;
    pcl::PointCloud<pcl::PointXYZI>* cloud;
    IO::read_pointcloud_from_xyz_file(cloud, "/home/jannik/Bachelorarbeit/data/cloud1.xyz");
    
    // pcl::VoxelGrid<pcl::PointXYZI> vg;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    // vg.setInputCloud (cloud->makeShared());
    // vg.setLeafSize (0.1f, 0.1f, 0.1f);
    // vg.filter (*cloud_filtered);
    // std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*

    *cloud_filtered = *cloud;

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZI> ());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZI> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    int i=0, nr_points = (int) cloud_filtered->size ();
    while (cloud_filtered->size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;
    }

    writer.write<pcl::PointXYZI> ("/home/jannik/Bachelorarbeit/data/clusters/after_planes.pcd", *cloud_filtered, false); 

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (0.3); // 2cm
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    std::cout << "clusters: " << cluster_indices.size() << std::endl;

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->push_back ((*cloud_filtered)[*pit]);
            cloud_cluster->width = cloud_cluster->size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
            std::stringstream ss;
            ss << "/home/jannik/Bachelorarbeit/data/clusters/cloud_cluster_" << j << ".pcd";
            writer.write<pcl::PointXYZI> (ss.str (), *cloud_cluster, false); 
            j++;
    }
}

// States
void Mapping::init_states() {
    this->use_rosbag = true;
    this->input_is_meter = false;
    this->input_is_lefthanded = false;
    this->script_path = ros::package::getPath("grabe_mapping") + "/scripts";
    this->cancelled = false;
    this->next_process = &Mapping::finish_mapping;
}

std::vector<std::string> Mapping::check_states() {

    std::vector<std::string> error_msgs;

    if(this->use_rosbag && this->rosbag_filename.isEmpty()) {
        error_msgs.push_back("ROSBAG: no rosbag filename set");
    }
    if(this->use_rosbag && this->scan_topic.isEmpty()) {
        error_msgs.push_back("TOPICS: no scan topic set");
    }
    if(this->use_rosbag && this->odom_topic.isEmpty()) {
        error_msgs.push_back("TOPICS: no odom topic set");
    }
    if(export_points && this->export_path.isEmpty()) {
        error_msgs.push_back("GENERAL: no export path set");
    }
    if(this->dir_path.isEmpty()) {
        error_msgs.push_back("OUTPUT: no directory path set");
    }
    if(this->start < 0) {
        error_msgs.push_back("GENERAL: start < 0");
    }
    if(this->end > -1 && this->start >= this->end) {
        error_msgs.push_back("GENERAL: start >= end");
    }
    if(this->improve_start != -1 && (this->improve_start < this->start || this->improve_start >= this->end)) {
        error_msgs.push_back("IMPROVE: start out of bounds");
    } 
    if(this->improve_end != -1 && (this->improve_end <= this->start || this->improve_end > this->end)) {
        error_msgs.push_back("IMPROVE: end out of bounds");
    }
    if(this->improve_start != -1 && this->improve_end != -1 && this->improve_end <= this->improve_start) {
        error_msgs.push_back("IMPROVE: start >= end");
    }
    if(this->max_it_ICP < 1) {
        error_msgs.push_back("ICP: max iterations < 1");
    }
    if(this->max_p2p_dist_ICP <= 0.0) {
        error_msgs.push_back("ICP: max p2p distance <= 0.0");
    }
    if(this->type_SLAM != 0 && this->max_it_SLAM < 1) {
        error_msgs.push_back("GRAPHSLAM: max iterations SLAM < 1");
    }
    if(this->type_SLAM != 0 && this->max_p2p_dist_SLAM <= 0.0) {
        error_msgs.push_back("GRAPHSLAM: max p2p distance SLAM <= 0.0");
    }
    if(this->type_Loop != 0 && this->max_it_Loop < 1) {
        error_msgs.push_back("GRAPHSLAM: max iterations Loop < 1");
    }
    if(this->type_Loop != 0 && this->max_p2p_dist_Loop <= 0.0) {
        error_msgs.push_back("GRAPHSLAM: max p2p distance Loop <= 0.0");
    }
    if(this->type_Loop != 0 && this->max_dist_Loop <= 0.0) {
        error_msgs.push_back("GRAPHSLAM: max distance Loop <= 0.0");
    }
    if(this->type_Loop != 0 && this->loopsize <= 0) {
        error_msgs.push_back("GRAPHSLAM: loopsize <= 0");
    }
    if(this->min_overlap_Loop == 0) {
        error_msgs.push_back("GRAPHSLAM: min overlap Loop = 0");
    }
    if(this->bucket_size <= 0) {
        error_msgs.push_back("ICP: bucket_size <= 0");
    }
    if(this->max_dist != -1 && this->min_dist != -1 && this->min_dist >= this->max_dist) {
        error_msgs.push_back("GENERAL: Distance min >= max");
    } 

    return error_msgs;
}

// Slots
void Mapping::on_process_finished() {
    (this->*next_process)();
}

// work
std::vector<double> Mapping::get_icp_results() const {
    return this->icp_results;
}

// setter
    // rosbag
void Mapping::set_use_rosbag(bool state) {
    this->use_rosbag = state;
}

void Mapping::set_rosbag_filename(QString filename) {
    this->rosbag_filename = filename;
}

void Mapping::set_input_is_meter(bool input_is_meter) {
    this->input_is_meter = input_is_meter;
}

void Mapping::toggle_input_is_meter() {
    this->input_is_meter = !this->input_is_meter;
}

void Mapping::set_input_is_lefthanded(bool input_is_lefthanded) {
    this->input_is_lefthanded = input_is_lefthanded;
    
}

void Mapping::toggle_input_is_lefthanded() {
    this->input_is_lefthanded = !this->input_is_lefthanded;
}

    // topics
void Mapping::set_scan_topic(QString topic) {
    this->scan_topic = topic;
}

void Mapping::set_odom_topic(QString topic) {
    this->odom_topic = topic;
}

void Mapping::set_gps_topic(QString topic) {
    this->gps_topic = topic;
}

    // Parameters
void Mapping::set_export_path(QString text) {
    this->export_path = text;
}

void Mapping::set_dir_path(QString path) {
   this->dir_path = path;
   this->loopclose_path = path + "loopclose.pts"; 
}

void Mapping::set_start(int start) {
    this->start = start;
}

void Mapping::set_end(int end) {
    this->end = end;
}

void Mapping::set_improve_start(int start) {
    this->improve_start = start;
}

void Mapping::set_improve_end(int end) {
    this->improve_end = end;
}

bool Mapping::set_ICP_type(int type) {
    if(type != 1 && type != 2 && type != 6)
        return false;

    this->type_ICP = type;
    return true;
}

void Mapping::set_epsilon_ICP(double eps) {
    this->epsilon_ICP = eps;
}

void Mapping::set_max_it_ICP(int it) {
    this->max_it_ICP = it;
}

void Mapping::set_max_p2p_dist_ICP(double dist) {
    this->max_p2p_dist_ICP = dist;
}

bool Mapping::set_SLAM_type(int type) {
    if(type < 0 || type > 4)
        return false;

    this->type_SLAM = type;
    return true;
}

void Mapping::set_epsilon_SLAM(double eps) {
    this->epsilon_SLAM = eps;
}

void Mapping::set_max_it_SLAM(int it) {
    this->max_it_SLAM = it;
}

void Mapping::set_max_p2p_dist_SLAM(double dist) {
    this->max_p2p_dist_SLAM = dist;
}

void Mapping::set_max_p2p_dist_finalSLAM(double dist) {
    this->max_p2p_dist_finalSLAM = dist;
}

bool Mapping::set_Loop_type(int type) {
    if(type < 0 || type > 4 || type == 1)
        return false;

    this->type_Loop = type;
    return true;
}

void Mapping::set_max_it_Loop(int it) {
    this->max_it_Loop = it;
}

void Mapping::set_max_p2p_dist_Loop(double dist) {
    this->max_p2p_dist_Loop = dist;
}

void Mapping::set_max_dist_Loop(double dist) {
    this->max_dist_Loop = dist;
}

void Mapping::set_dist_finalLoop(double dist) {
    this->max_dist_finalLoop = dist;
}

void Mapping::set_Loopsize(int size) {
    this->loopsize = size;
}

void Mapping::set_min_overlap_Loop(int size) {
    this->min_overlap_Loop = size;
}

bool Mapping::set_nns_method(int method) {
    if(method != 0 && method != 1) 
        return false;

    this->nns_method = method;
    return true;
}

void Mapping::set_bucket_size(int size) {
    this->bucket_size = size;
}

bool Mapping::set_pairing_mode(int mode) {
    if(mode < 0 || mode > 2)
        return false;

    if(mode == 0) {
        this->pairing_mode = CLOSEST_POINT;
    } else if(mode == 1) {
        this->pairing_mode = CLOSEST_POINT_ALONG_NORMAL_SIMPLE;
    } else if(mode == 2) {
        this->pairing_mode = CLOSEST_PLANE_SIMPLE;
    }

    return true;
}

void Mapping::set_min_dist(double dist) {
    this->min_dist = dist;
}

void Mapping::set_max_dist(double dist) {
    this->max_dist = dist;
}

void Mapping::set_red_voxel_size(double size) {
    this->red_voxel_size = size;
}

void Mapping::set_octree_red(int pts_per_voxel) {
    this->octree_red = pts_per_voxel;
}

void Mapping::set_random_red(int every_nth_pt) {
    this->random_red = every_nth_pt;
}

void Mapping::set_quiet(bool quiet) {
    this->quiet = quiet;
}

void Mapping::set_match_meta(bool match) {
    this->match_meta = match;
}

void Mapping::set_extrapolate_pose(bool eP) {
    this->extrapolate_pose = eP;
}

void Mapping::set_scanserver(bool scanserver) {
    this->scanserver = scanserver;
}

void Mapping::set_anim(int use_every_nth) {
    this->anim = use_every_nth;
}

void Mapping::set_export_pts(bool export_pts) {
    this->export_points = export_pts;
}

}