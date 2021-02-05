#ifndef GRABE_MAPPING_MAPPING_MANAGER_H
#define GRABE_MAPPING_MAPPING_MANAGER_H

#include <QWidget>
#include <QtConcurrent/QtConcurrent>

#include "mapping/mapping.h"

namespace grabe_mapping {

struct Errors {
    std::vector<double> errors;
    std::vector<int> status; // -1 = worse, 0 = nothing, 1 = better
};

class Mapping_manager : public QWidget {
    Q_OBJECT

public:

    static int first_scan;  
    static int last_scan;

    Mapping_manager();
    void addMapping(Mapping* mapping);
    void start_mapping();
    void showResults();
    static int run_command(std::string command);
    void write_frames();

    // getter
    Mapping* latest() { return this->current; }
    Errors get_errors();
    
    // setter
    void set_dir_path(QString text);
    void set_export_path(QString text);

private:

    std::vector<Mapping*> mappings;
    Mapping* current = nullptr;
    Errors errors;


    QString dir_path;
    QString export_path;
    QString loopclose_path;
    bool scan_server = false;
    IOType file_format = UOS;

    QFutureWatcher<int> watcher;

signals:
    void finished_mapping(int exit);

private slots:
    void process_finished();
};

}

#endif