#ifndef GRABE_MAPPING_SCAN_TO_FILE_WIDGET_H
#define GRABE_MAPPING_SCAN_TO_FILE_WIDGET_H

#include <QtGui>
#include <QWidget>
#include <rqt_gui_cpp/plugin.h>
#include <grabe_mapping/ui_scan_to_file_widget.h>
#include "mapping/mapping.h"

namespace grabe_mapping {

class Scan_to_file_widget : public QWidget {
    Q_OBJECT

private:
    Ui::Scan_to_file_widget ui;
    Mapping* mapping;

public:

    Scan_to_file_widget(Mapping* mapping, QString title = "Scan_to_file_widget", QWidget *parent = nullptr);
        
    void save_settings(qt_gui_cpp::Settings& instance_settings) const;
    void restore_settings(const qt_gui_cpp::Settings& instance_settings);

public slots:

    void pb_fileDialog_pressed();
    void le_filePath_text_changed(QString text);
    void rb_lefthanded_toggled();
    void rb_meter_toggled();

    // topics
    void le_scan_text_changed(QString text);
    void le_odom_text_changed(QString text);
    void le_gps_text_changed(QString text);
    void le_scan_type_text_changed(QString text);
    void le_odom_type_text_changed(QString text);
    void le_gps_type_text_changed(QString text);

    // output
    void pb_output_pressed();
    void le_output_text_changed(QString text);
};

}

#endif
