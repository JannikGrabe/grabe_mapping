#include "rqt_plugin/parameter_widget.h"

namespace grabe_mapping {

Parameter_widget::Parameter_widget(Mapping* mapping, QString title, QWidget *parent) : 
    QWidget(parent), mapping(mapping) {
    
    this->ui.setupUi(this);

    this->ui.gb_parameters->setTitle(title);

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
        // general
    QObject::connect(this->ui.sb_total, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &Parameter_widget::sb_total_value_changed);
    QObject::connect(this->ui.sb_first, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &Parameter_widget::sb_first_value_changed);
    QObject::connect(this->ui.sb_last, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &Parameter_widget::sb_last_value_changed);
    QObject::connect(this->ui.dsb_min, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &Parameter_widget::dsb_min_value_changed);
    QObject::connect(this->ui.dsb_max, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &Parameter_widget::dsb_max_value_changed);
    QObject::connect(this->ui.cb_correspondances, &QComboBox::currentTextChanged, this, &Parameter_widget::cb_correspondances_current_text_changed);
    QObject::connect(this->ui.cb_metascan, &QCheckBox::stateChanged, this, &Parameter_widget::cb_metascan_state_changed);

        // ICP
    QObject::connect(this->ui.cb_icp_minimization, &QComboBox::currentTextChanged, this, &Parameter_widget::cb_icp_minimization_current_text_changed);
    QObject::connect(this->ui.cb_nn, &QComboBox::currentTextChanged, this, &Parameter_widget::cb_nn_current_text_changed);
    QObject::connect(this->ui.sb_icp_iterations, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &Parameter_widget::sb_icp_iterations_value_changed);
    QObject::connect(this->ui.dsb_icp_epsilon, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &Parameter_widget::dsb_icp_epsilon_value_changed);
    QObject::connect(this->ui.dsb_nn_p2p_distance, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &Parameter_widget::dsb_nn_p2p_distance_value_changed);

        // graphSLAM  
    QObject::connect(this->ui.cb_closing_loop, &QComboBox::currentTextChanged, this, &Parameter_widget::cb_closing_loop_current_text_changed);
    QObject::connect(this->ui.cb_graphslam, &QComboBox::currentTextChanged, this, &Parameter_widget::cb_graphslam_current_text_changed);
    QObject::connect(this->ui.sb_loop_size, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &Parameter_widget::sb_loop_size_value_changed);
    QObject::connect(this->ui.sb_cl_max_distance, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &Parameter_widget::sb_cl_max_distance_value_changed);
    QObject::connect(this->ui.dsb_cl_p2p_distance, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &Parameter_widget::dsb_cl_p2p_distance_value_changed);
    QObject::connect(this->ui.sb_cl_iterations, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &Parameter_widget::sb_cl_iterations_value_changed); 
    QObject::connect(this->ui.sb_slam_iterations, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &Parameter_widget::sb_slam_iterations_value_changed); 
    QObject::connect(this->ui.dsb_graph_epsilon, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &Parameter_widget::dsb_graph_epsilon_value_changed);
    QObject::connect(this->ui.dsb_graph_p2p_distance, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &Parameter_widget::dsb_graph_p2p_distance_value_changed);
}

void Parameter_widget::save_settings(qt_gui_cpp::Settings& instance_settings) const {
    // general
    instance_settings.setValue("total", this->ui.sb_total->value());
    instance_settings.setValue("first_scan", this->ui.sb_first->value());
    instance_settings.setValue("last_scan", this->ui.sb_last->value());
    instance_settings.setValue("min_distance", this->ui.dsb_min->value());
    instance_settings.setValue("max_distance", this->ui.dsb_max->value());
    instance_settings.setValue("correspondances", this->ui.cb_correspondances->currentIndex());
    instance_settings.setValue("metascan", this->ui.cb_metascan->isChecked());
    // ICP
    instance_settings.setValue("icp_minimization", this->ui.cb_icp_minimization->currentIndex());
    instance_settings.setValue("nearest_neighbor", this->ui.cb_nn->currentIndex());
    instance_settings.setValue("icp_iterations", this->ui.sb_icp_iterations->value());
    instance_settings.setValue("icp_epsilon", this->ui.dsb_icp_epsilon->value());
    instance_settings.setValue("nn_max_p2p_distance", this->ui.dsb_nn_p2p_distance->value());
    // GraphSLAM
    instance_settings.setValue("closing_loop", this->ui.cb_closing_loop->currentIndex());
    instance_settings.setValue("graphslam", this->ui.cb_graphslam->currentIndex());
    instance_settings.setValue("loop_size", this->ui.sb_loop_size->value());
    instance_settings.setValue("cl_max_distance", this->ui.sb_cl_max_distance->value());
    instance_settings.setValue("cl_p2p_distance", this->ui.dsb_cl_p2p_distance->value());
    instance_settings.setValue("cl_iterations", this->ui.sb_cl_iterations->value());
    instance_settings.setValue("slam_iterations", this->ui.sb_slam_iterations->value());
    instance_settings.setValue("slam_epsilon", this->ui.dsb_graph_epsilon->value());
    instance_settings.setValue("slam_p2p_distance", this->ui.dsb_graph_p2p_distance->value());
}

void Parameter_widget::restore_settings(const qt_gui_cpp::Settings& instance_settings) {
    // general
    if(instance_settings.contains("total"))
        this->ui.sb_total->setValue(instance_settings.value("total").toInt());
    if(instance_settings.contains("first_scan"))
        this->ui.sb_first->setValue(instance_settings.value("first_scan").toInt());
    if(instance_settings.contains("last_scan"))
        this->ui.sb_last->setValue(instance_settings.value("last_scan").toInt());
    if(instance_settings.contains("min_distance"))
        this->ui.dsb_min->setValue(instance_settings.value("min_distance").toDouble());
    if(instance_settings.contains("max_distance"))
        this->ui.dsb_max->setValue(instance_settings.value("max_distance").toDouble());
    if(instance_settings.contains("correspondances"))
        this->ui.cb_correspondances->setCurrentIndex(instance_settings.value("correspondances").toInt());
    if(instance_settings.contains("metascan"))
        this->ui.cb_metascan->setChecked(instance_settings.value("metascan").toBool());
    // ICP
    if(instance_settings.contains("icp_minimization"))
        this->ui.cb_icp_minimization->setCurrentIndex(instance_settings.value("icp_minimization").toInt());
    if(instance_settings.contains("nearest_neighbor"))
        this->ui.cb_nn->setCurrentIndex(instance_settings.value("nearest_neighbor").toInt());
    if(instance_settings.contains("icp_iterations"))
        this->ui.sb_icp_iterations->setValue(instance_settings.value("icp_iterations").toInt());
    if(instance_settings.contains("icp_epsilon"))
        this->ui.dsb_icp_epsilon->setValue(instance_settings.value("icp_epsilon").toDouble());
    if(instance_settings.contains("nn_max_p2p_distance"))
        this->ui.dsb_nn_p2p_distance->setValue(instance_settings.value("nn_max_p2p_distance").toDouble());
    // GraphSLAM
    if(instance_settings.contains("closing_loop"))
        this->ui.cb_closing_loop->setCurrentIndex(instance_settings.value("closing_loop").toInt());
    if(instance_settings.contains("graphslam"))
        this->ui.cb_graphslam->setCurrentIndex(instance_settings.value("graphslam").toInt());
    if(instance_settings.contains("loop_size"))
        this->ui.sb_loop_size->setValue(instance_settings.value("loop_size").toInt());
    if(instance_settings.contains("cl_max_distance"))
        this->ui.sb_cl_max_distance->setValue(instance_settings.value("cl_max_distance").toInt());
    if(instance_settings.contains("cl_p2p_distance"))
        this->ui.dsb_cl_p2p_distance->setValue(instance_settings.value("cl_p2p_distance").toDouble());
    if(instance_settings.contains("cl_iterations"))
        this->ui.sb_cl_iterations->setValue(instance_settings.value("cl_iterations").toInt());
    if(instance_settings.contains("slam_iterations"))
        this->ui.sb_slam_iterations->setValue(instance_settings.value("slam_iterations").toInt());
    if(instance_settings.contains("slam_epsilon"))
        this->ui.dsb_graph_epsilon->setValue(instance_settings.value("slam_epsilon").toDouble());
    if(instance_settings.contains("slam_p2p_distance"))
        this->ui.dsb_graph_p2p_distance->setValue(instance_settings.value("slam_p2p_distance").toDouble());
}

// slots

// general
void Parameter_widget::sb_total_value_changed(int val) {
    if(val <= 1 ) {
        this->ui.sb_first->setValue(0);
        this->ui.sb_last->setValue(1);
        this->ui.sb_first->setEnabled(false);
        this->ui.sb_last->setEnabled(false);
    } else {
        this->ui.sb_first->setEnabled(true);
        this->ui.sb_last->setEnabled(true);
        this->ui.sb_last->setValue(val - 1);
    }
}

void Parameter_widget::sb_first_value_changed(int val) {
    int total = this->ui.sb_total->value();
    int last = this->ui.sb_last->value(); 

    if(val > total - 2) {
        this->ui.sb_first->setValue(0);
    } else if(val < 0) {
        this->ui.sb_first->setValue(last - 1);
    } else {
        this->mapping->set_start(val);

        if(val >= last) { // first is larger than last
        this->ui.sb_last->setValue(val + 1);
        }
    }
}

void Parameter_widget::sb_last_value_changed(int val) {
    int total = this->ui.sb_total->value();
    int first = this->ui.sb_first->value();

    if(val < 1) {
        this->ui.sb_last->setValue(total - 1);
    } else if (val > total - 1) {
        this->ui.sb_last->setValue(first + 1);
    } else{
        this->mapping->set_end(val);

        if(val < first) { // last is smaller than first
        this->ui.sb_first->setValue(val - 1);// set first one lower than this
        }
    }
}

void Parameter_widget::dsb_min_value_changed(double val) {
    double max = this->ui.dsb_max->value();
    
    if(val < 0.0) { // set inactive
        this->mapping->set_min_dist(-1);
    } else {
        this->mapping->set_min_dist(val);

        if(max >= 0.0 && val >= max) // first is larger than last
        this->ui.dsb_max->setValue(val + 1.0);
    }
}

void Parameter_widget::dsb_max_value_changed(double val) {
    static double previous = -1;
    double min = this->ui.dsb_min->value();

    if(val < 0.0) { // set inactive
        this->mapping->set_max_dist(-1);
        previous = val;
    } else {
        this->mapping->set_max_dist(val);

        if(min >= 0.0 && val <= min) { // last is smaller than first
        if(previous < 0.0) {            // was inactive -> set to one higher than first
            this->ui.dsb_max->setValue(min + 1.0);
            return;
        } else {                         // wasn't inactive -> set first one lower than this
            this->ui.dsb_min->setValue(val - 1.0);
        }
    }
        previous = val;
    }
}

void Parameter_widget::cb_correspondances_current_text_changed(QString text) {
    int index = this->ui.cb_correspondances->findText(text);
    int param_value = this->ui.cb_correspondances->itemData(index).toInt();

    if(!this->mapping->set_pairing_mode(param_value)) {
        QMessageBox::critical(this, "Warning", "Could not find " + text, QMessageBox::Ok);
    }
}

void Parameter_widget::cb_metascan_state_changed(int state) {
    this->mapping->set_match_meta(state);
}

// ICP
void Parameter_widget::cb_icp_minimization_current_text_changed(QString text) {
    int index = this->ui.cb_icp_minimization->findText(text);
    int param_value = this->ui.cb_icp_minimization->itemData(index).toInt();

    if(!this->mapping->set_ICP_type(param_value)) {
        QMessageBox::critical(this, "Warning", "Could not find " + text, QMessageBox::Ok);
    }
}

void Parameter_widget::cb_nn_current_text_changed(QString text) {
    int index = this->ui.cb_nn->findText(text);
    int param_value = this->ui.cb_nn->itemData(index).toInt();

    if(!this->mapping->set_nns_method(param_value)) {
        QMessageBox::critical(this, "Warning", "Could not find " + text, QMessageBox::Ok);
    }
}

void Parameter_widget::sb_icp_iterations_value_changed(int val) {
    this->mapping->set_max_it_ICP(val);
}

void Parameter_widget::dsb_icp_epsilon_value_changed(double val) {
    this->mapping->set_epsilon_ICP(val);
}

void Parameter_widget::dsb_nn_p2p_distance_value_changed(double val) {
    this->mapping->set_max_p2p_dist_ICP(val);
}

// GraphSLAM
void Parameter_widget::cb_closing_loop_current_text_changed(QString text) {
    int index = this->ui.cb_closing_loop->findText(text);
    int param_value = this->ui.cb_closing_loop->itemData(index).toInt();

    if(!this->mapping->set_Loop_type(param_value)) {
        QMessageBox::critical(this, "Warning", "Could not find " + text, QMessageBox::Ok);
    }
}

void Parameter_widget::cb_graphslam_current_text_changed(QString text) {
    int index = this->ui.cb_graphslam->findText(text);
    int param_value = this->ui.cb_graphslam->itemData(index).toInt();

    if(!this->mapping->set_SLAM_type(param_value)) {
        QMessageBox::critical(this, "Warning", "Could not find " + text, QMessageBox::Ok);
    } 
}

void Parameter_widget::sb_loop_size_value_changed(int val) {
    this->mapping->set_Loopsize(val);
}

void Parameter_widget::sb_cl_max_distance_value_changed(int val) {
    this->mapping->set_max_dist_Loop(val);
}

void Parameter_widget::dsb_cl_p2p_distance_value_changed(double val) {
    this->mapping->set_max_p2p_dist_Loop(val);
}

void Parameter_widget::sb_cl_iterations_value_changed(int val) {
    this->mapping->set_max_it_Loop(val);
}

void Parameter_widget::sb_slam_iterations_value_changed(int val) {
    this->mapping->set_max_it_SLAM(val);
}

void Parameter_widget::dsb_graph_epsilon_value_changed(double val) {
    this->mapping->set_epsilon_SLAM(val);
}

void Parameter_widget::dsb_graph_p2p_distance_value_changed(double val) {
    this->mapping->set_max_p2p_dist_SLAM(val);
}

}