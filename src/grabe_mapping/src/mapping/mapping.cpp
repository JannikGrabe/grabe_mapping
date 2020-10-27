#include "mapping/mapping.h"
#include "ros/ros.h"
#include "ros/package.h"

Mapping::Mapping() {

    this->init_states();

    QObject::connect(&this->watcher, &QFutureWatcher<void>::finished, this, &Mapping::on_work_finished);
}

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
}

int Mapping::run_command(std::string command) {
    return system(command.c_str());
}

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

void Mapping::on_work_finished() {
    emit this->finished_mapping(0);
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

    // topic
QString Mapping::get_scan_topic() {
    return this->scan_topic;
}

QString Mapping::get_odom_topic() {
    return this->odom_topic;
}

QString Mapping::get_gps_topic() {
    return this->gps_topic;
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
    std::cout << "meter: " << this->input_is_meter << std::endl;
}

void Mapping::set_input_is_lefthanded(bool input_is_lefthanded) {
    this->input_is_lefthanded = input_is_lefthanded;
    
}

void Mapping::toggle_input_is_lefthanded() {
    this->input_is_lefthanded = !this->input_is_lefthanded;
    std::cout << "lefthanded: " << this->input_is_lefthanded << std::endl;
}

    // topic
void Mapping::set_scan_topic(QString topic) {
    this->scan_topic = topic;
}

void Mapping::set_odom_topic(QString topic) {
    this->odom_topic = topic;
}

void Mapping::set_gps_topic(QString topic) {
    this->gps_topic = topic;
}

    // output
void Mapping::set_output_filepath(QString filename) {
    this->output_filepath = filename;
}

