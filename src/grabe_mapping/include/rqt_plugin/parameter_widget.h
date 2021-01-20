#ifndef GRABE_MAPPING_PARAMETER_WIDGET_H
#define GRABE_MAPPING_PARAMETER_WIDGET_H

#include <QtGui>
#include <QWidget>
#include <QMessageBox>
#include <grabe_mapping/ui_parameter_widget.h>
#include "mapping/mapping.h"

namespace grabe_mapping {

class Parameter_widget : public QWidget {
    Q_OBJECT

private:

    Ui::Parameter_widget ui;
    Mapping* mapping;

public:
    Parameter_widget(Mapping* mapping, QString title = "Parameter_widget", QWidget *parent = nullptr) : 
        QWidget(parent), mapping(mapping) {
        
        this->ui.setupUi(this);

        this->setWindowTitle(title);

        // initialize Comboboxes
        this->ui.cb_correspondances->addItem("closest point", QVariant(0));
        this->ui.cb_correspondances->addItem("closest along normal", QVariant(1));
        this->ui.cb_correspondances->addItem("closest point-to-plane distance", QVariant(2));

        this->ui.cb_icp_minimization->addItem("Unit Quaternion", QVariant(1)); 
        this->ui.cb_icp_minimization->addItem("Singular Value Decomposition", QVariant(2));
        //this->ui_.cb_minimization->addItem("Orthonormal Matrices");
        //this->ui_.cb_minimization->addItem("Dual Quaternions");
        //this->ui_.cb_minimization->addItem("Helix Approximation");
        this->ui.cb_icp_minimization->addItem("Small Angle Approximation", QVariant(6));
        //this->ui_.cb_minimization->addItem("Uncertainty Based: Euler Angles");
        //this->ui_.cb_minimization->addItem("Uncertainty Based: Quaternions"); 
        //this->ui_.cb_minimization->addItem("Unit Quaternion with Scale Method");
                            
        this->ui.cb_nn->addItem("simple k-d tree", QVariant(0)); 
        this->ui.cb_nn->addItem("cached k-d tree", QVariant(1));
        //this->ui_.cb_nn->addItem("ANN tree");
        //this->ui_.cb_nn->addItem("BOC tree");

        this->ui.cb_closing_loop->addItem("no loop closing", QVariant(0));
        //this->ui_.cb_closing_loop->addItem("Euler Angles", QVariant(1));
        this->ui.cb_closing_loop->addItem("Quaternions", QVariant(2));
        this->ui.cb_closing_loop->addItem("Unit Quaternions", QVariant(3));
        this->ui.cb_closing_loop->addItem("SLERP", QVariant(4));

        this->ui.cb_graphslam->addItem("no GraphSLAM", QVariant(0));
        this->ui.cb_graphslam->addItem("Euler Angles", QVariant(1));
        this->ui.cb_graphslam->addItem("Unit Quaternions", QVariant(2));
        this->ui.cb_graphslam->addItem("Helix Approximation", QVariant(3));
        this->ui.cb_graphslam->addItem("Small Angle Approximation", QVariant(4));

        // connect slots:
            // GENERAL
        QObject::connect(this->ui.sb_first, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this->mapping, &Mapping::set_start);
        QObject::connect(this->ui.sb_last, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this->mapping, &Mapping::set_end);
        QObject::connect(this->ui.dsb_min, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this->mapping, &Mapping::set_min_dist);
        QObject::connect(this->ui.dsb_max, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this->mapping, &Mapping::set_max_dist);
        QObject::connect(this->ui.cb_correspondances, &QComboBox::currentTextChanged, this, &Parameter_widget::on_cb_correspondances_current_text_changed);
        QObject::connect(this->ui.cb_metascan, &QCheckBox::stateChanged, this->mapping, &Mapping::set_match_meta);
            // ICP
        QObject::connect(this->ui.cb_icp_minimization, &QComboBox::currentTextChanged, this, &Parameter_widget::on_cb_icp_minimization_current_text_changed);
        QObject::connect(this->ui.cb_nn, &QComboBox::currentTextChanged, this, &Parameter_widget::on_cb_nn_current_text_changed);
        QObject::connect(this->ui.sb_icp_iterations, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this->mapping, &Mapping::set_max_it_ICP);
        QObject::connect(this->ui.dsb_icp_epsilon, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this->mapping, &Mapping::set_epsilon_ICP);
        QObject::connect(this->ui.dsb_nn_p2p_distance, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this->mapping, &Mapping::set_max_p2p_dist_ICP);
            // SLAM
        QObject::connect(this->ui.cb_closing_loop, &QComboBox::currentTextChanged, this, &Parameter_widget::on_cb_closing_loop_current_text_changed);
        QObject::connect(this->ui.cb_graphslam, &QComboBox::currentTextChanged, this, &Parameter_widget::on_cb_graphslam_current_text_changed);
        QObject::connect(this->ui.sb_loop_size, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this->mapping, &Mapping::set_Loopsize);
        QObject::connect(this->ui.sb_cl_max_distance, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this->mapping, &Mapping::set_max_dist_Loop);
        QObject::connect(this->ui.sb_cl_max_distance, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this->mapping, &Mapping::set_max_dist_finalLoop);
        QObject::connect(this->ui.dsb_cl_p2p_distance, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this->mapping, &Mapping::set_max_p2p_dist_Loop);
        QObject::connect(this->ui.sb_cl_iterations, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this->mapping, &Mapping::set_max_it_Loop); 
        QObject::connect(this->ui.sb_slam_iterations, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this->mapping, &Mapping::set_max_it_SLAM); 
        QObject::connect(this->ui.dsb_graph_epsilon, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this->mapping, &Mapping::set_epsilon_SLAM);
        QObject::connect(this->ui.dsb_graph_p2p_distance, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this->mapping, &Mapping::set_max_p2p_dist_SLAM);
    }

public slots:
    void on_cb_correspondances_current_text_changed(const QString& text) {

        int index = this->ui.cb_correspondances->findText(text);
        int param_value = this->ui.cb_correspondances->itemData(index).toInt();

        if(!this->mapping->set_ICP_type(param_value)) {
            QMessageBox::critical(this, "Warning", "Could not find " + text, QMessageBox::Ok);
        }
    }

    void on_cb_icp_minimization_current_text_changed(const QString& text) {

        int index = this->ui.cb_icp_minimization->findText(text);
        int param_value = this->ui.cb_icp_minimization->itemData(index).toInt();

        if(!this->mapping->set_ICP_type(param_value)) {
            QMessageBox::critical(this, "Warning", "Could not find " + text, QMessageBox::Ok);
        }
    }

    void on_cb_nn_current_text_changed(const QString& text) {
        int index = this->ui.cb_nn->findText(text);
        int param_value = this->ui.cb_nn->itemData(index).toInt();

        if(!this->mapping->set_nns_method(param_value)) {
            QMessageBox::critical(this, "Warning", "Could not find " + text, QMessageBox::Ok);
        }
    }

    void on_cb_closing_loop_current_text_changed(const QString& text) {
        int index = this->ui.cb_closing_loop->findText(text);
        int param_value = this->ui.cb_closing_loop->itemData(index).toInt();

        if(!this->mapping->set_Loop_type(param_value)) {
            QMessageBox::critical(this, "Warning", "Could not find " + text, QMessageBox::Ok);
        }
    }

    void on_cb_graphslam_current_text_changed(const QString& text) {
        int index = this->ui.cb_graphslam->findText(text);
        int param_value = this->ui.cb_graphslam->itemData(index).toInt();

        if(!this->mapping->set_SLAM_type(param_value)) {
            QMessageBox::critical(this, "Warning", "Could not find " + text, QMessageBox::Ok);
        } 
    }
};

}

#endif