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
        return;
    } else if(this->dir_path.isEmpty() || !QDir(this->dir_path).exists()) {
        std::cout << "GENERAL no directory set" << std::endl;
    }

    this->current->lock_parameters();

    int start = this->current->get_start();
    int end = this->current->get_end();

    if(this->mappings.size() == 1) {
        first_scan = start;
        last_scan = end;

        Scan::openDirectory(this->scan_server, this->dir_path.toStdString(), this->file_format, first_scan, last_scan);
    }

    if(start < first_scan || start >= last_scan) {
        std::cout << "IMPROV invalid start" << std::endl;
        return;
    } else if(end <= first_scan || end > last_scan) {
        std::cout << "IMPROV invalid end" << std::endl;
        return;
    }
    
    QFuture<int> future = QtConcurrent::run(this->current, &Mapping::start_slam6d);
    this->watcher.setFuture(future);
}

void Mapping_manager::showResults() {

    // build command to run show
    std::ostringstream oss(ros::package::getPath("grabe_mapping") + "/bin/show ", std::ios_base::app);
    oss << "-s " << first_scan << " -e " << last_scan << " " << this->dir_path.toStdString();

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