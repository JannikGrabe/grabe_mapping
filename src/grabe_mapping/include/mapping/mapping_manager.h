#ifndef GRABE_MAPPING_MAPPING_MANAGER_H
#define GRABE_MAPPING_MAPPING_MANAGER_H

#include "mapping/mapping.h"

namespace grabe_mapping {

class Mapping_manager {

public:

    static int first_scan;  
    static int last_scan;

    void addMapping(Mapping* mapping);
    
    //setter
    void set_dir_path(QString text);
    void set_export_path(QString text);

private:

    std::vector<Mapping*> mappings;

    QString dir_path;
    QString export_path;
    QString loopclose_path;
    bool scan_server = false;
    IOType file_format = UOS;
};

}

#endif