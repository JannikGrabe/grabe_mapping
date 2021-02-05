#include "mapping/mapping_manager.h"
#include <ros/package.h>

namespace grabe_mapping {

int Mapping_manager::first_scan = 0;
int Mapping_manager::last_scan = 1;

Mapping_manager::Mapping_manager() {
    QObject::connect(&this->watcher, &QFutureWatcher<int>::finished, this, &Mapping_manager::process_finished);
}

void Mapping_manager::addMapping(Mapping* mapping) {
    this->mappings.push_back(mapping);
    this->current = mapping;
}

void Mapping_manager::start_mapping() {
    
    if(current == nullptr) {
        std::cout << "GENERAL no mapping set" << std::endl;
        emit finished_mapping(-1);
        return;
    } else if(this->dir_path.isEmpty()) {
        std::cout << "GENERAL no directory set" << std::endl;
        emit finished_mapping(-1);
        return;
    }

    this->current->lock_parameters();

    int start = this->current->get_start();
    int end = this->current->get_end();

    if(start >= end) {
        std::cout << "GENERAL start >= end" << std::endl;
        emit finished_mapping(-1);
        return;
    }
    
    if(this->mappings.size() == 1) {
        first_scan = start;
        last_scan = end;

        Scan::openDirectory(this->scan_server, this->dir_path.toStdString(), this->file_format, first_scan, last_scan);
    
        if(Scan::allScans.size() == 0) {
            std::cout << "GENERAL no scans found" << std::endl;
            emit finished_mapping(-1);
            return;
        }
    }

    if(start < first_scan || start >= last_scan) {
        std::cout << "IMPROV invalid start" << std::endl;
        emit finished_mapping(-1);
        return;
    } else if(end <= first_scan || end > last_scan) {
        std::cout << "IMPROV invalid end" << std::endl;
        emit finished_mapping(-1);
        return;
    }
    
    QFuture<int> future = QtConcurrent::run(this->current, &Mapping::start_slam6d);
    this->watcher.setFuture(future);
}

void Mapping_manager::showResults() {

    // build command to run show
    std::ostringstream oss(ros::package::getPath("grabe_mapping") + "/bin/show ", std::ios_base::app);
    oss << "-s " << this->current->get_start(true) << " -e " << this->current->get_end(true) << " " << this->dir_path.toStdString();

    std::cout << oss.str() << std::endl;

    QFuture<int> show_future = QtConcurrent::run(Mapping_manager::run_command, oss.str());
}

int Mapping_manager::run_command(std::string command) {
    return system(command.c_str());
}

void Mapping_manager::write_frames() {
    
    if(this->dir_path.isEmpty()) {
        std::cout << "GENERAL no directory set" << std::endl;
        return;
    }
    
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

Errors Mapping_manager::get_errors() {
    
    std::vector<double> new_errors = this->current->get_errors();
    
    if(this->errors.errors.size() == 0) {
        this->errors.errors = new_errors;
        this->errors.status = std::vector<int>(new_errors.size(), 0);
        return this->errors;
    }

    int start = this->current->get_start();
    int end = this->current->get_end();

    for(int i = start - Mapping_manager::first_scan, j = 0; j < new_errors.size(); j++) {
        if(this->errors.errors[i] == new_errors[j]) {
            this->errors.status[i] = 0;
        } else if( this->errors.errors[i] < new_errors[j]) {
            this->errors.status[i] = -1;
        } else {
            this->errors.status[i] = 1;
        }
        this->errors.errors[i] = new_errors[j];
    }

    return this->errors;
}

void Mapping_manager::set_dir_path(QString text) {
    this->dir_path = text;
    this->loopclose_path = text + "loopclose.pts";
}

void Mapping_manager::set_export_path(QString text) {
    this->export_path = text;
}

void Mapping_manager::process_finished() {
    emit this->finished_mapping(this->watcher.result());
}
    
}