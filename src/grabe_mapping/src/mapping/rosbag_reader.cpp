#include "mapping/rosbag_reader.h"
#include "ros/package.h"
#include <iostream>

namespace grabe_mapping {

Rosbag_reader::Rosbag_reader() {
    QObject::connect(&this->watcher, &QFutureWatcher<void>::finished, this, &Rosbag_reader::on_finished);
}

void Rosbag_reader::read() {

    this->cancelled = false;

    std::vector<std::string> errors = this->check_states();
    if(this->check_states().size() > 0) {
        for(int i = 0; i < errors.size(); i++) {
            std::cout << errors[i] << std::endl;
        }
        emit this->finished();
        return;
    }

    std::string exe = ros::package::getPath("grabe_mapping") + "/scripts/scan_to_file_launch.sh ";

    exe += this->rosbag_filename.toStdString();                                            // rosbag_filename
    exe += " " + this->scan_topic.toStdString();                                           // scan_topic name
    exe += " " + this->odom_topic.toStdString();                                           // odom_topic name
    exe += " " + (this->input_is_meter ? std::string("true") : std::string("false"));      // input_is_meter flag
    exe += " " + (this->input_is_lefthanded ? std::string("true") : std::string("false")); // input_is_lefthanded flags
    exe += " " + this->output_path.toStdString(); 
    
    std::cout << exe << std::endl;                                     // path where to put any output files

    emit this->started();

    QFuture<int> future = QtConcurrent::run(Rosbag_reader::run_command, exe);

    this->watcher.setFuture(future);
}

void Rosbag_reader::cancel() {
    this->cancelled = true;
    Rosbag_reader::run_command("rosnode kill /player");
}

std::vector<std::string> Rosbag_reader::check_states() {
    std::vector<std::string> error_msgs;

    if(this->rosbag_filename.isEmpty()) {
        error_msgs.push_back("ROSBAG: rosbag filename not set");
    }
    if(this->scan_topic.isEmpty()) {
        error_msgs.push_back("ROSBAG: scan topic not set");
    }
    if(this->odom_topic.isEmpty()) {
        error_msgs.push_back("ROSBAG: odom topic not set");
    }
    // if(this->gps_topic.isEmpty()) {
    //     error_msgs.push_back("ROSBAG: gps topic not set");
    // }

    return error_msgs;
}

int Rosbag_reader::run_command(std::string command) {
    return system(command.c_str());
}

// slots

void Rosbag_reader::on_finished() {
    if(!this->cancelled)
        emit this->output_path_changed(this->output_path);

    emit this->finished();
}

// setter

void Rosbag_reader::set_rosbag_filename(QString filename) {
    this->rosbag_filename = filename;
}

void Rosbag_reader::set_output_path(QString path) {
    this->output_path = path;
}

void Rosbag_reader::set_scan_topic(QString topic) {
    this->scan_topic = topic;
}

void Rosbag_reader::set_odom_topic(QString topic) {
    this->odom_topic = topic;
}

void Rosbag_reader::set_gps_topic(QString topic) {
    this->gps_topic = topic;
}

void Rosbag_reader::toggle_input_is_meter() {
    this->input_is_meter = !this->input_is_meter;
}

void Rosbag_reader::toggle_input_is_lefthanded() {
    this->input_is_lefthanded = !this->input_is_lefthanded;
}


}