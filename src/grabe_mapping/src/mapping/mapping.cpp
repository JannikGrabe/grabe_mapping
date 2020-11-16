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

    this->next_process = &Mapping::finish_scan_to_file;
}

void Mapping::finish_scan_to_file() {
    emit this->finished_rosbag();
    this->next_process = &Mapping::start_slam6D;
    this->on_process_finished();
}

void Mapping::start_slam6D() {
    std::string slam6D = "/home/jannik/slam6d-code/bin/slam6D";

    slam6D += this->parameters->to_string();

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
    if(this->cancelled) {
        emit this->finished_mapping(1);
        this->cancelled = false;
    } else {
        emit this->finished_mapping(0);

        bool export_points = false;
        this->parameters->get_is_active("--exportAllPoints", export_points);

        if(export_points) {
            std::string move_export = "mv " + QDir::current().absolutePath().toStdString() + "/points.pts "
                                    + this->export_path.toStdString();
            
            this->run_command(move_export);
        }
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

// States
void Mapping::init_states() {
    this->use_rosbag = true;
    this->input_is_meter = false;
    this->input_is_lefthanded = false;
    this->script_path = ros::package::getPath("grabe_mapping") + "/scripts";
    this->cancelled = false;
    this->next_process = &Mapping::finish_mapping;
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
    }

    return true;
}

// Algorithms
void Mapping::initAlgorithms() {
    
    this->parameters = new grabe_mapping::Parameter_map();

    // ICP 
    parameters->add_parameter("-a", 1);
    parameters->add_parameter("-i", 50);
    parameters->add_parameter("--epsICP", 0.000001);
    
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
    parameters->add_parameter("-s", -1, false);
    parameters->add_parameter("-e", -1, false);
    parameters->add_parameter("--max", -1.0, false);
    parameters->add_parameter("--min", -1.0, false);
    parameters->add_parameter("--normal-shoot-simple", false);
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

