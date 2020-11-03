#include "mapping/mapping.h"
#include "ros/ros.h"
#include "ros/package.h"
#include <QProcess>

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

    this->next_process = &Mapping::start_slam6D;
}

void Mapping::start_slam6D() {
    std::string slam6D = "/home/jannik/slam6d-code/bin/slam6D";

    for(std::map<std::string, MappingAlgorithm*>::iterator it = this->algorithms.begin(); it != this->algorithms.end(); it++) {
        slam6D += it->second->to_string();
    }

    slam6D += " " + this->output_filepath.toStdString();

    std::cout << slam6D << std::endl;

    QFuture<int> slam6D_future = QtConcurrent::run(Mapping::run_command, slam6D);

    this->watcher.setFuture(slam6D_future);

    this->next_process = &Mapping::finish_mapping;
}

void Mapping::showResults() {

    // build command to run show
    std::string show = "/home/jannik/slam6d-code/bin/show ";
    show += this->output_filepath.toStdString();

    QFuture<int> show_future = QtConcurrent::run(Mapping::run_command, show);

    this->watcher.setFuture(show_future);

    // this->next_process = &Mapping::finish_mapping;
}

void Mapping::finish_mapping() {
    emit this->finished_mapping(0);
}

int Mapping::run_command(std::string command) {
    return system(command.c_str());
}

void Mapping::cancel_mapping() {
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
    
    // ICP 
    MappingAlgorithm* icp_minimization = new MappingAlgorithm("icp_minimization");
    icp_minimization->add_parameter("-a", 1);
    icp_minimization->add_parameter("-i", 50);
    icp_minimization->add_parameter("--epsICP", 0.000001);

    this->algorithms.insert(std::pair<std::string, MappingAlgorithm*>("icp_minimization", icp_minimization));
    
    // nearest neighbor
    MappingAlgorithm* nearest_neighbor = new MappingAlgorithm("nearest_neighbor");
    nearest_neighbor->add_parameter("-t", 0);
    nearest_neighbor->add_parameter("--dist", 25);
    
    this->algorithms.insert(std::pair<std::string, MappingAlgorithm*>("nearest_neighbor", nearest_neighbor));

    // closing loop
    MappingAlgorithm* closing_loop = new MappingAlgorithm("closing_loop");
    closing_loop->add_parameter("-L", 0);
    closing_loop->add_parameter("--loopsize", 20);
    closing_loop->add_parameter("--cldist", 500);
    closing_loop->add_parameter("--clpairs", -1, false);
    closing_loop->add_parameter("--distLoop", 700);
    closing_loop->add_parameter("--iterLoop", 100);

    this->algorithms.insert(std::pair<std::string, MappingAlgorithm*>("closing_loop", closing_loop));

    // graphslam minimization
    MappingAlgorithm* graph_slam = new MappingAlgorithm("graph_slam");
    graph_slam->add_parameter("-G", 0);
    graph_slam->add_parameter("-I", 50);
    graph_slam->add_parameter("--epsSLAM", 0.5);
    graph_slam->add_parameter("--distSLAM", 25);

    this->algorithms.insert(std::pair<std::string, MappingAlgorithm*>("graph_slam", graph_slam));

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

    // Algorithms

bool Mapping::set_algorithm_parameter(std::string algorithm_name, std::string parameter_name, double parameter_value) {
    if(this->algorithms.find(algorithm_name) == this->algorithms.end()) {
        return false;
    }

    return this->algorithms[algorithm_name]->set_parameter(parameter_name, parameter_value);
}

    // output
void Mapping::set_output_filepath(QString filename) {
    this->output_filepath = filename;
}

