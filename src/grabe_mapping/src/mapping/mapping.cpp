#include "mapping/mapping.h"
#include "ros/ros.h"
#include "ros/package.h"
#include <QProcess>

Mapping::Mapping() {

    this->init_states();

    this->initAlgorithms();    

    //QObject::connect(&this->watcher, &QFutureWatcher<void>::finished, this, &Mapping::on_process_finished);
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

    this->next_process = &Mapping::start_slam6D;
}

void Mapping::start_slam6D() {
    std::string slam6D = "/home/jannik/slam6d-code/bin/slam6D ";

    slam6D += this->minimization.to_string();
    slam6D += " " + this->nearest_neighbor.to_string();
    slam6D += " " + this->output_filepath.toStdString();

    std::cout << slam6D << std::endl;

    QFuture<int> slam6D_future = QtConcurrent::run(Mapping::run_command, slam6D);

    this->watcher.setFuture(slam6D_future);

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

void Mapping::cancel_mapping() {

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

// States
void Mapping::init_states() {
    this->use_rosbag = true;
    this->input_is_meter = false;
    this->input_is_lefthanded = false;
    this->script_path = ros::package::getPath("grabe_mapping") + "/scripts";
}

bool Mapping::check_states() {
    if(this->use_rosbag && this->rosbag_filename.isEmpty()) {
        ROS_ERROR("no rosbag filename set");
        return false;
    } else if(this->use_rosbag && this->scan_topic.isEmpty()) {
        ROS_ERROR("no scan topic set");
        return false;
    } else if(this->use_rosbag && this->odom_topic.isEmpty()) {
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
    this->minimization = MappingAlgorithm("Unit Quaternion", "-a", 1);

    this->minimization_algorithms.insert(std::pair<std::string, MappingAlgorithm>(
        "Unit Quaternion", this->minimization));
    this->minimization_algorithms.insert(std::pair<std::string, MappingAlgorithm>(
        "Singular Value Decomposition", MappingAlgorithm("Singular Value Decomposition", "-a", 2)));
    this->minimization_algorithms.insert(std::pair<std::string, MappingAlgorithm>(
        "Orthonormal Matrices", MappingAlgorithm("Orthonormal Matrices", "-a", 3)));
    this->minimization_algorithms.insert(std::pair<std::string, MappingAlgorithm>(
        "Dual Quaternions", MappingAlgorithm("Dual Quaternions", "-a", 4)));
    this->minimization_algorithms.insert(std::pair<std::string, MappingAlgorithm>(
        "Helix Approximation", MappingAlgorithm("Helix Approximation", "-a", 5)));
    this->minimization_algorithms.insert(std::pair<std::string, MappingAlgorithm>(
        "Small Angle Approximation", MappingAlgorithm("Small Angle Approximation", "-a", 6)));
    this->minimization_algorithms.insert(std::pair<std::string, MappingAlgorithm>(
        "Uncertainty Based: Euler Angles", MappingAlgorithm("Uncertainty Based: Euler Angles", "-a", 7)));
    this->minimization_algorithms.insert(std::pair<std::string, MappingAlgorithm>(
        "Uncertainty Based: Quaternions", MappingAlgorithm("Uncertainty Based: Quaternions", "-a", 8)));
    this->minimization_algorithms.insert(std::pair<std::string, MappingAlgorithm>(
        "Unit Quaternion with Scale Method", MappingAlgorithm("Unit Quaternion with Scale Method", "-a", 9)));

    // ICP Nearest Neighbor
    this->nearest_neighbor = MappingAlgorithm("simple k-d tree", "-t", 0);

    this->nearest_neighbor_algorithms.insert(std::pair<std::string, MappingAlgorithm>(
        "simple k-d tree", this->nearest_neighbor));
    this->nearest_neighbor_algorithms.insert(std::pair<std::string, MappingAlgorithm>(
        "cached k-d tree", MappingAlgorithm("cached k-d tree", "-t", 2)));
    this->nearest_neighbor_algorithms.insert(std::pair<std::string, MappingAlgorithm>(
        "ANN tree", MappingAlgorithm("ANN tree", "-t", 3)));
    this->nearest_neighbor_algorithms.insert(std::pair<std::string, MappingAlgorithm>(
        "BOC tree", MappingAlgorithm("BOC tree", "-t", 4))); 
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

    // ICP
MappingAlgorithm Mapping::get_minimization() const {
    return this->minimization;
}

MappingAlgorithm Mapping::get_nearest_neighbor() const {
    return this->nearest_neighbor;
}

    // output
QString Mapping::get_output_filepath() const {
    return this->output_filepath;
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

    // ICP
bool Mapping::set_minimization(QString text) {
    std::map<std::string,MappingAlgorithm>::iterator it = this->minimization_algorithms.find(text.toStdString());
    if(it != this->minimization_algorithms.end()) {
        this->minimization = it->second;
        return true;
    } else {
        return false;
    }
}

bool Mapping::set_nearest_neighbor(QString text) {
    std::map<std::string,MappingAlgorithm>::iterator it = this->nearest_neighbor_algorithms.find(text.toStdString());
    if(it != this->nearest_neighbor_algorithms.end()) {
        this->nearest_neighbor = it->second;
        return true;
    } else {
        return false;
    }
}

    // output
void Mapping::set_output_filepath(QString filename) {
    this->output_filepath = filename;
}

