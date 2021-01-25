#include "mapping/mapping_manager.h"

namespace grabe_mapping {

void Mapping_manager::addMapping(Mapping* mapping) {
    this->mappings.push_back(mapping);
}

void Mapping_manager::set_dir_path(QString text) {
    this->dir_path = text;
}

void Mapping_manager::set_export_path(QString text) {
    this->export_path = text;
}

}