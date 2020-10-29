#include "mapping/mapping.h"
#include "ros/ros.h"
#include "ros/package.h"

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

    this->start_scan_to_file();
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

    this->next_process = &Mapping::showResults;
}

void Mapping::showResults() {

    // build command to run show
    std::string show = "/home/jannik/slam6d-code/bin/show -s 0 -e 12 ";
    show += this->output_filepath.toStdString();

    QFuture<int> show_future = QtConcurrent::run(Mapping::run_command, show);

    this->watcher.setFuture(show_future);

    this->next_process = &Mapping::finish_mapping;
}

void Mapping::finish_mapping() {
    emit this->finished_mapping(0);
}

int Mapping::run_command(std::string command) {
    return system(command.c_str());
}


// States
void Mapping::init_states() {
    this->input_is_meter = false;
    this->input_is_lefthanded = false;
    this->script_path = ros::package::getPath("grabe_mapping") + "/scripts";
}

bool Mapping::check_states() {
    if(this->rosbag_filename.isEmpty()) {
        ROS_ERROR("no rosbag filename set");
        return false;
    } else if(this->scan_topic.isEmpty()) {
        ROS_ERROR("no scan topic set");
        return false;
    } else if(this->odom_topic.isEmpty()) {
        ROS_ERROR("no odom topic set");
        return false;
    } /*else if(this->gps_topic.isEmpty()) {
        ROS_ERROR("no gps topic set");
        return false;
    }*/ else if(this->output_filepath.isEmpty()) {
        ROS_ERROR("no output filepath set");
        return false;
    }

    return true;
}

// Algorithms
void Mapping::initAlgorithms() {
    // ICP Minimization
    this->minimization_algorithms.insert(std::pair<std::string, MappingAlgorithm>(
        "Unit Quaternion", MappingAlgorithm("Unit Quaternion")));
    this->minimization_algorithms.insert(std::pair<std::string, MappingAlgorithm>(
        "Singular Value Decomposition", MappingAlgorithm("Singular Value Decomposition")));
    this->minimization_algorithms.insert(std::pair<std::string, MappingAlgorithm>(
        "Orthonormal Matrices", MappingAlgorithm("Orthonormal Matrices")));
    this->minimization_algorithms.insert(std::pair<std::string, MappingAlgorithm>(
        "Dual Quaternion", MappingAlgorithm("Dual Quaternions")));
    this->minimization_algorithms.insert(std::pair<std::string, MappingAlgorithm>(
        "Helix Approximation", MappingAlgorithm("Helix Approximation")));
    this->minimization_algorithms.insert(std::pair<std::string, MappingAlgorithm>(
        "Small Angle Approximation", MappingAlgorithm("Small Angle Approximation")));
    this->minimization_algorithms.insert(std::pair<std::string, MappingAlgorithm>(
        "Uncertainty Based: Euler Angles", MappingAlgorithm("Uncertainty Based: Euler Angles")));
    this->minimization_algorithms.insert(std::pair<std::string, MappingAlgorithm>(
        "Uncertainty Based: Quaternions", MappingAlgorithm("Uncertainty Based: Quaternions")));
    this->minimization_algorithms.insert(std::pair<std::string, MappingAlgorithm>(
        "Unit Quaternion with Scale Method", MappingAlgorithm("Unit Quaternion with Scale Method")));

    // ICP Nearest Neighbor
    this->nearest_neighbor_algorithms.insert(std::pair<std::string, MappingAlgorithm>(
        "simple k-d tree", MappingAlgorithm("simple k-d tree")));
    this->nearest_neighbor_algorithms.insert(std::pair<std::string, MappingAlgorithm>(
        "cached k-d tree", MappingAlgorithm("cached k-d tree")));
    this->nearest_neighbor_algorithms.insert(std::pair<std::string, MappingAlgorithm>(
        "ANN tree", MappingAlgorithm("ANN tree")));
    this->nearest_neighbor_algorithms.insert(std::pair<std::string, MappingAlgorithm>(
        "BOC tree", MappingAlgorithm("BOC tree"))); 
}

// Slots
void Mapping::on_process_finished() {
    (this->*next_process)();
}

// getter
    // rosbag
QString Mapping::get_rosbag_filename() {
    return this->rosbag_filename;
}

bool Mapping::get_input_is_meter() {
    return this->input_is_meter;
}

bool Mapping::get_input_is_lefthanded() {
    return this->input_is_lefthanded;
}

    // topics
QString Mapping::get_scan_topic() {
    return this->scan_topic;
}

QString Mapping::get_odom_topic() {
    return this->odom_topic;
}

QString Mapping::get_gps_topic() {
    return this->gps_topic;
}

    // ICP
MappingAlgorithm Mapping::get_minimization() {
    return this->minimization;
}

MappingAlgorithm Mapping::get_nearest_neighbor() {
    return this->nearest_neighbor;
}

    // output
QString Mapping::get_output_filepath() {
    return this->output_filepath;
}

// setter
    // rosbag
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

    // ICP
void Mapping::set_minimization(QString text) {
    this->minimization = this->minimization_algorithms.find(text.toStdString())->second;
}

void Mapping::set_nearest_neighbor(QString text) {
    this->nearest_neighbor = this->nearest_neighbor_algorithms.find(text.toStdString())->second;
}

    // output
void Mapping::set_output_filepath(QString filename) {
    this->output_filepath = filename;
}

