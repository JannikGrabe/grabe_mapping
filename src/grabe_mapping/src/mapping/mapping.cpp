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


Mapping::Mapping() {

    this->init_states();

    this->initAlgorithms();   

    QObject::connect(&this->watcher, &QFutureWatcher<void>::finished, this, &Mapping::on_process_finished);
}

// Mapping
void Mapping::start_mapping() {
    if(!this->check_states()) {
        emit this->finished_mapping(1);
        return;
    }

    if(this->use_rosbag) 
        this->start_scan_to_file();
    else
        this->start_slam6D();
}

void Mapping::start_scan_to_file() {
    // build command to run script to launch scan_to_file_node
    std::string scan_to_file_launch_sh = this->script_path + "/scan_to_file_launch.sh ";
    scan_to_file_launch_sh += this->rosbag_filename.toStdString();                                            // rosbag_filename
    scan_to_file_launch_sh += " " + this->scan_topic.toStdString();                                           // scan_topic name
    scan_to_file_launch_sh += " " + this->odom_topic.toStdString();                                           // odom_topic name
    scan_to_file_launch_sh += " " + (this->input_is_meter ? std::string("true") : std::string("false"));      // input_is_meter flag
    scan_to_file_launch_sh += " " + (this->input_is_lefthanded ? std::string("true") : std::string("false")); // input_is_lefthanded flags
    scan_to_file_launch_sh += " " + this->output_filepath.toStdString();                                      // path where to put any output files
  
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
    //std::string slam6D = "/home/jannik/slam6d-code/bin/slam6D";
    std::string slam6D = ros::package::getPath("three_dtk") + "/bin/slam6D";

    slam6D += this->parameters->to_string();

    slam6D += " " + this->output_filepath.toStdString();

    slam6D += " | tee " + this->output_filepath.toStdString() + "/results.txt";

    std::cout << slam6D << std::endl;

    QFuture<int> slam6D_future = QtConcurrent::run(Mapping::run_command, slam6D);

    this->watcher.setFuture(slam6D_future);

    this->next_process = &Mapping::finish_mapping;
}

void Mapping::showResults() {

    // build command to run show
    std::string show = "/home/jannik/slam6d-code/bin/show ";
    show += this->parameters->to_string("-s");
    show += this->parameters->to_string("-e");
    show += this->output_filepath.toStdString();

    QFuture<int> show_future = QtConcurrent::run(Mapping::run_command, show);

    this->watcher.setFuture(show_future);

    // this->next_process = &Mapping::finish_mapping;
}

void Mapping::finish_mapping() {
    if(this->cancelled) {
        emit this->finished_mapping(1);
        this->cancelled = false;
    } else {
        
        bool export_points = false;
        this->parameters->get_is_active("--exportAllPoints", export_points);

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

    std::ifstream results_file((this->output_filepath + "/results.txt").toStdString().c_str());

    if(!results_file.is_open()) {
        std::cout << "could not open results file" << std::endl;
    }
    std::string line_new = "";
    std::string line_old = "";

    while(std::getline(results_file, line_new)) {
        std::string first_word;

        std::stringstream ss(line_new);
        ss >> first_word;

        if(first_word.compare("TIME") == 0) {
            std::istringstream iss(line_old);
            std::vector<std::string> words((std::istream_iterator<std::string>(iss)),
                                            std::istream_iterator<std::string>());

            this->icp_results.push_back(words[5]);
            this->icp_results.push_back(words[7]);
        }
        line_old = line_new;
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

void Mapping::stopChildProcesses(qint64 parentProcessId) {
    
    QProcess get_children;
    QStringList get_children_cmd;
    get_children_cmd << "--ppid" << QString::number(parentProcessId) << "-o" << "pid" << "--no-heading";
    get_children.start("ps", get_children_cmd);
    get_children.waitForFinished();
    QString childIds(get_children.readAllStandardOutput());
    childIds.replace('\n', ' ');

    //QProcess::execute("kill " + childIds);
    QProcess kill_children;
    kill_children.start("kill " + childIds);
    kill_children.waitForFinished();
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
    std::ostringstream scan1_base(this->output_filepath.toStdString() + "/scan", std::ios_base::app);
    scan1_base << std::setfill('0') << std::setw(3) << scan1;

    std::ostringstream scan2_base(this->output_filepath.toStdString() + "/scan", std::ios_base::app);
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
    std::ostringstream scan1_base(this->output_filepath.toStdString() + "/scan", std::ios_base::app);
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
    this->file_count = 0;
}

bool Mapping::check_states() {

    bool export_points = false;
    this->parameters->get_is_active("--exportAllPoints", export_points);

    if(this->use_rosbag && this->rosbag_filename.isEmpty()) {
        ROS_ERROR("no rosbag filename set");
        return false;
    } else if(this->use_rosbag && this->scan_topic.isEmpty()) {
        ROS_ERROR("no scan topic set");
        return false;
    } else if(this->use_rosbag && this->odom_topic.isEmpty()) {
        ROS_ERROR("no odom topic set");
        return false;
    } else if(export_points && this->export_path.isEmpty()) {
        ROS_ERROR("no export path set");
        return false;
    } else if(this->output_filepath.isEmpty()) {
        ROS_ERROR("no output filepath set");
        return false;
    } else if(!this->use_rosbag && this->file_count <= 1) {
        ROS_ERROR("no total number of scans set");
        return false;
    }

    return true;
}

// Algorithms
void Mapping::initAlgorithms() {
    
    this->parameters = new grabe_mapping::Parameter_map();

    // ICP 
    parameters->add_parameter("-a", 1);
    parameters->add_parameter("-i", 50);
    parameters->add_parameter("--epsICP", 0.00001);
    
    // nearest neighbor
    parameters->add_parameter("-t", 0);
    parameters->add_parameter("--dist", 25.0);

    // closing loop
    parameters->add_parameter("-L", 0);
    parameters->add_parameter("--loopsize", 20);
    parameters->add_parameter("--cldist", 500);
    parameters->add_parameter("--clpairs", -1, false);
    parameters->add_parameter("--distLoop", 700.0);
    parameters->add_parameter("--iterLoop", 100);

    // graphslam minimization
    parameters->add_parameter("-G", 0);
    parameters->add_parameter("-I", 50);
    parameters->add_parameter("--epsSLAM", 0.5);
    parameters->add_parameter("--distSLAM", 25.0);

    // general
    parameters->add_parameter("-s", 0);
    parameters->add_parameter("-e", 1);
    parameters->add_parameter("--max", -1.0, false);
    parameters->add_parameter("--min", -1.0, false);
    parameters->add_parameter("--normal_shoot-simple", false);
    parameters->add_parameter("--point-to-plane-simple", false);
    parameters->add_parameter("--exportAllPoints", false);
    parameters->add_parameter("--metascan", false);
    parameters->add_parameter("--loopclosefile", (this->output_filepath + "loopclose.pts").toStdString());
}

// Slots
void Mapping::on_process_finished() {
    (this->*next_process)();
}

// getter
    // rosbag
bool Mapping::get_use_rosbag() const{
    return this->use_rosbag;
}

QString Mapping::get_rosbag_filename() const {
    return this->rosbag_filename;
}

bool Mapping::get_input_is_meter() const {
    return this->input_is_meter;
}

bool Mapping::get_input_is_lefthanded() const {
    return this->input_is_lefthanded;
}

int Mapping::get_file_count() const {
    return this->file_count;
}

    // topics
QString Mapping::get_scan_topic() const {
    return this->scan_topic;
}

QString Mapping::get_odom_topic() const {
    return this->odom_topic;
}

QString Mapping::get_gps_topic() const {
    return this->gps_topic;
}

    // output
QString Mapping::get_output_filepath() const {
    return this->output_filepath;
}

    // work
std::vector<std::string> Mapping::get_icp_results() const {
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

void Mapping::set_file_count(int val) {
    if(val >= 0) 
        this->file_count = val;
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

bool Mapping::set_parameter_value(std::string name, double value) {
    return this->parameters->set_value(name, value);
}

bool Mapping::set_parameter_value(std::string name, int value) {
    return this->parameters->set_value(name, value);
}

bool Mapping::set_parameter_value(std::string name, std::string value) {
    return this->parameters->set_value(name, value);
}

bool Mapping::set_parameter_active(std::string name, bool state) {
    return this->parameters->set_active(name, state);
}

bool Mapping::toggle_parameter_active(std::string name) {
    return this->parameters->toggle_active(name);
}

void Mapping::set_export_path(QString text) {
    this->export_path = text;
}

    // output
void Mapping::set_output_filepath(QString filename) {
    this->output_filepath = filename;
    this->set_parameter_value("--loopclosefile", (this->output_filepath + "/loopclose.pts").toStdString());
}

