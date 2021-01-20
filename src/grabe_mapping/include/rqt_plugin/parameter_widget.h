#ifndef GRABE_MAPPING_PARAMETER_WIDGET_H
#define GRABE_MAPPING_PARAMETER_WIDGET_H

#include <QtGui>
#include <QWidget>
#include <QMessageBox>
#include <rqt_gui_cpp/plugin.h>
#include <grabe_mapping/ui_parameter_widget.h>
#include "mapping/mapping.h"

namespace grabe_mapping {

class Parameter_widget : public QWidget {
    Q_OBJECT

private:

    Ui::Parameter_widget ui;
    Mapping* mapping;

public:
    Parameter_widget(Mapping* mapping, QString title = "Parameter_widget", QWidget *parent = nullptr);
    
    void save_settings(qt_gui_cpp::Settings& instance_settings) const;
    void restore_settings(const qt_gui_cpp::Settings& instance_settings);

public slots:
    // general
    void sb_total_value_changed(int val);
    void sb_first_value_changed(int val);
    void sb_last_value_changed(int val);
    void dsb_min_value_changed(double val);
    void dsb_max_value_changed(double val);
    void cb_correspondances_current_text_changed(QString text);
    void cb_metascan_state_changed(int state);
    // ICP
    void cb_icp_minimization_current_text_changed(QString text);
    void cb_nn_current_text_changed(QString text);
    void sb_icp_iterations_value_changed(int val);
    void dsb_icp_epsilon_value_changed(double val);
    void dsb_nn_p2p_distance_value_changed(double val);
    // GraphSLAM
    void cb_closing_loop_current_text_changed(QString text);
    void cb_graphslam_current_text_changed(QString text);
    void sb_loop_size_value_changed(int val);
    void sb_cl_max_distance_value_changed(int val);
    void dsb_cl_p2p_distance_value_changed(double val);
    void sb_cl_iterations_value_changed(int val);
    void sb_slam_iterations_value_changed(int val);
    void dsb_graph_epsilon_value_changed(double val);
    void dsb_graph_p2p_distance_value_changed(double val);
    
};

}

#endif