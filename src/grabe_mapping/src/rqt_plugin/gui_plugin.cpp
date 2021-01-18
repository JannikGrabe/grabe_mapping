
#include "rqt_plugin/gui_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QtConcurrent/QtConcurrent>
#include <QStringList>
#include <QFileDialog>
#include <QLineEdit>
#include <QProgressBar>
#include <QMessageBox>
#include <QSettings>
#include <QTableView>

namespace grabe_mapping
{

GuiPlugin::GuiPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("GuiPlugin");
}

void GuiPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QMainWindow();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface

  QCoreApplication::setOrganizationName("GrabeMapping");
  QCoreApplication::setOrganizationDomain("grabe-mapping.grb");
  QCoreApplication::setApplicationName("Grabe Mapping");

  this->count_sub = this->n.subscribe("mapping/scan_to_file_count", 10, &GuiPlugin::scan_to_file_count_callback, this);

  this->mapping = new Mapping();

  this->initWidgets();
  
  context.addWidget(widget_);

  // rosbag
  QObject::connect(ui_.cb_update_scans, &QCheckBox::stateChanged, this, &GuiPlugin::on_cb_update_scans_state_changed);
  QObject::connect(ui_.pb_fileDialog, &QPushButton::pressed, this, &GuiPlugin::on_pb_fileDialog_pressed);
  QObject::connect(ui_.rb_lefthanded, &QRadioButton::toggled, this, &GuiPlugin::on_rb_lefthanded_toggled);
  QObject::connect(ui_.rb_meter, &QRadioButton::toggled, this, &GuiPlugin::on_rb_meter_toggled);
  QObject::connect(ui_.le_filePath, &QLineEdit::textChanged, this, &GuiPlugin::on_le_filePath_text_changed);

  // topics
  QObject::connect(ui_.le_scan, &QLineEdit::textChanged, this, &GuiPlugin::on_le_scan_text_changed);
  QObject::connect(ui_.le_odom, &QLineEdit::textChanged, this, &GuiPlugin::on_le_odom_text_changed);
  QObject::connect(ui_.le_gps, &QLineEdit::textChanged, this, &GuiPlugin::on_le_gps_text_changed);

  // output
  QObject::connect(ui_.pb_output, &QPushButton::pressed, this, &GuiPlugin::on_pb_output_pressed);
  QObject::connect(ui_.le_output, &QLineEdit::textChanged, this, &GuiPlugin::on_le_output_text_changed);

  // work
  QObject::connect(this->mapping, &Mapping::finished_mapping, this, &GuiPlugin::on_work_finished);
  QObject::connect(this->mapping, &Mapping::finished_rosbag, this, &GuiPlugin::on_rosbag_finished);
  QObject::connect(ui_.pb_start, &QPushButton::pressed, this, &GuiPlugin::on_pb_start_pressed);
  QObject::connect(ui_.pb_show, &QPushButton::pressed, this, &GuiPlugin::on_pb_show_pressed);
  QObject::connect(this->ui_.pb_cancel, &QPushButton::pressed, this, &GuiPlugin::on_pb_cancel_pressed);
  QObject::connect(this->ui_.pb_back, &QPushButton::pressed, this, &GuiPlugin::on_pb_back_pressed);


  // general
  QObject::connect(this->ui_.sb_total, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &GuiPlugin::on_sb_total_value_changed);
  QObject::connect(this->ui_.sb_first, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &GuiPlugin::on_sb_first_value_changed);
  QObject::connect(this->ui_.sb_last, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &GuiPlugin::on_sb_last_value_changed);
  QObject::connect(this->ui_.dsb_min, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &GuiPlugin::on_dsb_min_value_changed);
  QObject::connect(this->ui_.dsb_max, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &GuiPlugin::on_dsb_max_value_changed);
  QObject::connect(this->ui_.cb_correspondances, &QComboBox::currentTextChanged, this, &GuiPlugin::on_cb_correspondances_current_text_changed);
  QObject::connect(this->ui_.cb_metascan, &QCheckBox::stateChanged, this, &GuiPlugin::on_cb_metascan_state_changed);
  QObject::connect(this->ui_.cb_export, &QCheckBox::stateChanged, this, &GuiPlugin::on_cb_export_state_changed);
  QObject::connect(this->ui_.le_export, &QLineEdit::textChanged, this, &GuiPlugin::on_le_export_text_changed);
  QObject::connect(this->ui_.pb_export, &QPushButton::pressed, this, &GuiPlugin::on_pb_export_pressed);

  // ICP
  QObject::connect(this->ui_.cb_icp_minimization, &QComboBox::currentTextChanged, this, &GuiPlugin::on_cb_icp_minimization_current_text_changed);
  QObject::connect(this->ui_.cb_nn, &QComboBox::currentTextChanged, this, &GuiPlugin::on_cb_nn_current_text_changed);
  QObject::connect(this->ui_.sb_icp_iterations, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &GuiPlugin::on_sb_icp_iterations_value_changed);
  QObject::connect(this->ui_.dsb_icp_epsilon, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &GuiPlugin::on_dsb_icp_epsilon_value_changed);
  QObject::connect(this->ui_.dsb_nn_p2p_distance, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &GuiPlugin::on_dsb_nn_p2p_distance_value_changed);

  // graphSLAM  
  QObject::connect(this->ui_.cb_closing_loop, &QComboBox::currentTextChanged, this, &GuiPlugin::on_cb_closing_loop_current_text_changed);
  QObject::connect(this->ui_.cb_graphslam, &QComboBox::currentTextChanged, this, &GuiPlugin::on_cb_graphslam_current_text_changed);
  QObject::connect(this->ui_.sb_loop_size, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &GuiPlugin::on_sb_loop_size_value_changed);
  QObject::connect(this->ui_.sb_cl_max_distance, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &GuiPlugin::on_sb_cl_max_distance_value_changed);
  QObject::connect(this->ui_.dsb_cl_p2p_distance, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &GuiPlugin::on_dsb_cl_p2p_distance_value_changed);
  QObject::connect(this->ui_.sb_cl_iterations, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &GuiPlugin::on_sb_cl_iterations_value_changed); 
  QObject::connect(this->ui_.sb_slam_iterations, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &GuiPlugin::on_sb_slam_iterations_value_changed); 
  QObject::connect(this->ui_.dsb_graph_epsilon, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &GuiPlugin::on_dsb_graph_epsilon_value_changed);
  QObject::connect(this->ui_.dsb_graph_p2p_distance, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &GuiPlugin::on_dsb_graph_p2p_distance_value_changed);
  
  // config
  QObject::connect(this->ui_.pb_save_config, &QPushButton::pressed, this, &GuiPlugin::on_pb_save_config_pressed);
  QObject::connect(this->ui_.pb_load_config, &QPushButton::pressed, this, &GuiPlugin::on_pb_load_config_pressed);
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

// slots
  // rosbag
void GuiPlugin::on_pb_fileDialog_pressed() {
  
  QString rosbag_filename = QFileDialog::getOpenFileName(
    widget_,
    tr("Select Bagfile"),
    "/home/jannik/Bachelorarbeit/Bagfiles",
    tr("*.bag")
  );

  this->ui_.le_filePath->setText(rosbag_filename);
}

void GuiPlugin::on_le_filePath_text_changed(QString text) {
  this->mapping->set_rosbag_filename(text);
}

void GuiPlugin::on_rb_lefthanded_toggled() {
  this->mapping->toggle_input_is_lefthanded();
}

void GuiPlugin::on_rb_meter_toggled() {
  this->mapping->toggle_input_is_meter();
}

  // topics
void GuiPlugin::on_le_scan_text_changed(QString text) {
  this->mapping->set_scan_topic(text);
}

void GuiPlugin::on_le_odom_text_changed(QString text) {
  this->mapping->set_odom_topic(text);
}

void GuiPlugin::on_le_gps_text_changed(QString text) {
  this->mapping->set_gps_topic(text);
}

void GuiPlugin::on_le_scan_type_text_changed(QString text) {

}
 
void GuiPlugin::on_le_odom_type_text_changed(QString text) {

}

void GuiPlugin::on_le_gps_type_text_changed(QString text) {

}

// general
void GuiPlugin::on_sb_total_value_changed(int val) {
  if(val <= 1 ) {
    this->ui_.sb_first->setValue(0);
    this->ui_.sb_last->setValue(1);
    this->ui_.sb_first->setEnabled(false);
    this->ui_.sb_last->setEnabled(false);
  } else {
    this->ui_.sb_first->setEnabled(true);
    this->ui_.sb_last->setEnabled(true);
    this->ui_.sb_last->setValue(val - 1);
  }
}

void GuiPlugin::on_sb_first_value_changed(int val) {
  int total = this->ui_.sb_total->value();
  int last = this->ui_.sb_last->value(); 

  if(val > total - 2) {
    this->ui_.sb_first->setValue(0);
  } else if(val < 0) {
    this->ui_.sb_first->setValue(last - 1);
  } else {
    this->mapping->set_start(val);

    if(val >= last) { // first is larger than last
      this->ui_.sb_last->setValue(val + 1);
    }
  }
}

void GuiPlugin::on_sb_last_value_changed(int val) {
  int total = this->ui_.sb_total->value();
  int first = this->ui_.sb_first->value();

  if(val < 1) {
    this->ui_.sb_last->setValue(total - 1);
  } else if (val > total - 1) {
    this->ui_.sb_last->setValue(first + 1);
  } else{
    this->mapping->set_end(val);

    if(val < first) { // last is smaller than first
      this->ui_.sb_first->setValue(val - 1);// set first one lower than this
    }
  }
}

void GuiPlugin::on_dsb_min_value_changed(double val) {
  double max = this->ui_.dsb_max->value();
  
  if(val < 0.0) { // set inactive
    this->mapping->set_min_dist(-1);
  } else {
    this->mapping->set_min_dist(val);

    if(max >= 0.0 && val >= max) // first is larger than last
      this->ui_.dsb_max->setValue(val + 1.0);
  }
}

void GuiPlugin::on_dsb_max_value_changed(double val) {
  static double previous = -1;
  double min = this->ui_.dsb_min->value();

  if(val < 0.0) { // set inactive
    this->mapping->set_max_dist(-1);
    previous = val;
  } else {
    this->mapping->set_max_dist(val);

    if(min >= 0.0 && val <= min) { // last is smaller than first
      if(previous < 0.0) {            // was inactive -> set to one higher than first
        this->ui_.dsb_max->setValue(min + 1.0);
        return;
      } else {                         // wasn't inactive -> set first one lower than this
        this->ui_.dsb_min->setValue(val - 1.0);
      }
  }
    previous = val;
  }
}

void GuiPlugin::on_cb_correspondances_current_text_changed(QString text) {
  if(text == "default") {
    this->mapping->set_pairing_mode(0);
  } else if(text == "closest along normal") {
    this->mapping->set_pairing_mode(1);
  } else if(text == "closest point-to-plane distance") {
    this->mapping->set_pairing_mode(2);
  }
}

void GuiPlugin::on_cb_metascan_state_changed(int state) {
  this->mapping->set_match_meta(state);
}

void GuiPlugin::on_cb_export_state_changed(int state) {
  this->mapping->set_export_pts(state);
  this->ui_.le_export->setEnabled(state);
  this->ui_.pb_export->setEnabled(state);
}

void GuiPlugin::on_pb_export_pressed() {
  QString filename = QFileDialog::getSaveFileName(
        this->widget_,
        "export file", 
        this->ui_.le_output->text() +  "/points.pts",
        tr("export file (*.pts)"));

  this->ui_.le_export->setText(filename);
}

void GuiPlugin::on_le_export_text_changed(QString text) {
  this->mapping->set_export_path(text);
}

// ICP
void GuiPlugin::on_cb_icp_minimization_current_text_changed(QString text) {

  int index = this->ui_.cb_icp_minimization->findText(text);
  int param_value = this->ui_.cb_icp_minimization->itemData(index).toInt();

  if(!this->mapping->set_ICP_type(param_value)) {
    QMessageBox::critical(this->widget_, "Warning", "Could not find " + text, QMessageBox::Ok);
  }
}

void GuiPlugin::on_cb_nn_current_text_changed(QString text) {
  int index = this->ui_.cb_nn->findText(text);
  int param_value = this->ui_.cb_nn->itemData(index).toInt();

  if(!this->mapping->set_nns_method(param_value)) {
    QMessageBox::critical(this->widget_, "Warning", "Could not find " + text, QMessageBox::Ok);
  }
}

void GuiPlugin::on_sb_icp_iterations_value_changed(int val) {

  this->mapping->set_max_it_ICP(val);
}

void GuiPlugin::on_dsb_icp_epsilon_value_changed(double val) {
    this->mapping->set_epsilon_ICP(val);

}

void GuiPlugin::on_dsb_nn_p2p_distance_value_changed(double val) {
    this->mapping->set_max_p2p_dist_ICP(val);
}

// GraphSLAM
void GuiPlugin::on_cb_closing_loop_current_text_changed(QString text) {
  int index = this->ui_.cb_closing_loop->findText(text);
  int param_value = this->ui_.cb_closing_loop->itemData(index).toInt();

  if(!this->mapping->set_Loop_type(param_value)) {
    QMessageBox::critical(this->widget_, "Warning", "Could not find " + text, QMessageBox::Ok);
  }
}

void GuiPlugin::on_cb_graphslam_current_text_changed(QString text) {
  int index = this->ui_.cb_graphslam->findText(text);
  int param_value = this->ui_.cb_graphslam->itemData(index).toInt();

  if(!this->mapping->set_SLAM_type(param_value)) {
    QMessageBox::critical(this->widget_, "Warning", "Could not find " + text, QMessageBox::Ok);
  } 
}

void GuiPlugin::on_sb_loop_size_value_changed(int val) {
  this->mapping->set_Loopsize(val);
}

void GuiPlugin::on_sb_cl_max_distance_value_changed(int val) {
  this->mapping->set_max_dist_Loop(val);
}

void GuiPlugin::on_dsb_cl_p2p_distance_value_changed(double val) {
  this->mapping->set_max_p2p_dist_Loop(val);
}

void GuiPlugin::on_sb_cl_iterations_value_changed(int val) {
  this->mapping->set_max_it_Loop(val);
}

void GuiPlugin::on_sb_slam_iterations_value_changed(int val) {
  this->mapping->set_max_it_SLAM(val);
}

void GuiPlugin::on_dsb_graph_epsilon_value_changed(double val) {
  this->mapping->set_epsilon_SLAM(val);
}

void GuiPlugin::on_dsb_graph_p2p_distance_value_changed(double val) {
  this->mapping->set_max_p2p_dist_SLAM(val);
}

// output
void GuiPlugin::on_pb_output_pressed() {
  QString output_filepath = QFileDialog::getExistingDirectory(
    widget_,
    tr("Select Directory for output files"),
    "/home/jannik/Bachelorarbeit",
    QFileDialog::ShowDirsOnly
  );

  this->ui_.le_output->setText(output_filepath);
}

void GuiPlugin::on_le_output_text_changed(QString text) {
  if(text.at(text.size() - 1) != '/')
    text += '/';
  this->mapping->set_dir_path(text);
}

// config
void GuiPlugin::on_pb_save_config_pressed() {
  QString filename = QFileDialog::getSaveFileName(this->widget_, "save config", "config.ini",
        tr("Config file (*.ini)"));

  QSettings settings(filename, QSettings::IniFormat, this->widget_);

  // rosbag
  settings.setValue("rosbag_filename", this->ui_.le_filePath->text());
  settings.setValue("input_is_meter", this->ui_.rb_meter->isChecked());
  settings.setValue("input_is_lefthanded", this->ui_.rb_lefthanded->isChecked());
  // topics
  settings.setValue("scan_topic", this->ui_.le_scan->text());
  settings.setValue("odom_topic", this->ui_.le_odom->text());
  settings.setValue("gps_topic", this->ui_.le_gps->text());
  // output
  settings.setValue("output_filepath", this->ui_.le_output->text());
  // general
  settings.setValue("total", this->ui_.sb_total->value());
  settings.setValue("first_scan", this->ui_.sb_first->value());
  settings.setValue("last_scan", this->ui_.sb_last->value());
  settings.setValue("min_distance", this->ui_.dsb_min->value());
  settings.setValue("max_distance", this->ui_.dsb_max->value());
  settings.setValue("correspondances", this->ui_.cb_correspondances->currentIndex());
  settings.setValue("metascan", this->ui_.cb_metascan->isChecked());
  settings.setValue("export", this->ui_.cb_export->isChecked());
  settings.setValue("export_path", this->ui_.le_export->text());
  // ICP
  settings.setValue("icp_minimization", this->ui_.cb_icp_minimization->currentIndex());
  settings.setValue("nearest_neighbor", this->ui_.cb_nn->currentIndex());
  settings.setValue("icp_iterations", this->ui_.sb_icp_iterations->value());
  settings.setValue("icp_epsilon", this->ui_.dsb_icp_epsilon->value());
  settings.setValue("nn_max_p2p_distance", this->ui_.dsb_nn_p2p_distance->value());
  // GraphSLAM
  settings.setValue("closing_loop", this->ui_.cb_closing_loop->currentIndex());
  settings.setValue("graphslam", this->ui_.cb_graphslam->currentIndex());
  settings.setValue("loop_size", this->ui_.sb_loop_size->value());
  settings.setValue("cl_max_distance", this->ui_.sb_cl_max_distance->value());
  settings.setValue("cl_p2p_distance", this->ui_.dsb_cl_p2p_distance->value());
  settings.setValue("cl_iterations", this->ui_.sb_cl_iterations->value());
  settings.setValue("slam_iterations", this->ui_.sb_slam_iterations->value());
  settings.setValue("slam_epsilon", this->ui_.dsb_graph_epsilon->value());
  settings.setValue("slam_p2p_distance", this->ui_.dsb_graph_p2p_distance->value());
}

void GuiPlugin::on_pb_load_config_pressed() {
  QString filename = QFileDialog::getOpenFileName(
    widget_,
    tr("Select Config File"),
    "",
    tr("*.ini")
  );

  QSettings instance_settings(filename, QSettings::IniFormat, this->widget_);
  
  // rosbag
  if(instance_settings.contains("rosbag_filename"))
    this->ui_.le_filePath->setText(instance_settings.value("rosbag_filename").toString());
  if(instance_settings.contains("input_is_meter"))
    this->ui_.rb_meter->setChecked(instance_settings.value("input_is_meter").toBool());
  if(instance_settings.contains("input_is_lefthanded"))
    this->ui_.rb_lefthanded->setChecked(instance_settings.value("input_is_lefthanded").toBool());
  // topics
  if(instance_settings.contains("scan_topic"))
    this->ui_.le_scan->setText(instance_settings.value("scan_topic").toString());
  if(instance_settings.contains("odom_topic"))
    this->ui_.le_odom->setText(instance_settings.value("odom_topic").toString());
  if(instance_settings.contains("gps_topic"))
    this->ui_.le_gps->setText(instance_settings.value("gps_topic").toString());
  // output
  if(instance_settings.contains("output_filepath"))
    this->ui_.le_output->setText(instance_settings.value("output_filepath").toString());
  // general
  if(instance_settings.contains("total"))
    this->ui_.sb_total->setValue(instance_settings.value("total").toInt());
  if(instance_settings.contains("first_scan"))
    this->ui_.sb_first->setValue(instance_settings.value("first_scan").toInt());
  if(instance_settings.contains("last_scan"))
    this->ui_.sb_last->setValue(instance_settings.value("last_scan").toInt());
  if(instance_settings.contains("min_distance"))
    this->ui_.dsb_min->setValue(instance_settings.value("min_distance").toDouble());
  if(instance_settings.contains("max_distance"))
    this->ui_.dsb_max->setValue(instance_settings.value("max_distance").toDouble());
  if(instance_settings.contains("correspondances"))
    this->ui_.cb_correspondances->setCurrentIndex(instance_settings.value("correspondances").toInt());
  if(instance_settings.contains("metascan"))
    this->ui_.cb_metascan->setChecked(instance_settings.value("metascan").toBool());
  if(instance_settings.contains("export"))
    this->ui_.cb_export->setChecked(instance_settings.value("export").toBool());
  if(instance_settings.contains("export_path"))
    this->ui_.le_export->setText(instance_settings.value("export_path").toString());
  // ICP
  if(instance_settings.contains("icp_minimization"))
    this->ui_.cb_icp_minimization->setCurrentIndex(instance_settings.value("icp_minimization").toInt());
  if(instance_settings.contains("nearest_neighbor"))
    this->ui_.cb_nn->setCurrentIndex(instance_settings.value("nearest_neighbor").toInt());
  if(instance_settings.contains("icp_iterations"))
    this->ui_.sb_icp_iterations->setValue(instance_settings.value("icp_iterations").toInt());
  if(instance_settings.contains("icp_epsilon"))
    this->ui_.dsb_icp_epsilon->setValue(instance_settings.value("icp_epsilon").toDouble());
  if(instance_settings.contains("nn_max_p2p_distance"))
    this->ui_.dsb_nn_p2p_distance->setValue(instance_settings.value("nn_max_p2p_distance").toDouble());
  // GraphSLAM
  if(instance_settings.contains("closing_loop"))
    this->ui_.cb_closing_loop->setCurrentIndex(instance_settings.value("closing_loop").toInt());
  if(instance_settings.contains("graphslam"))
    this->ui_.cb_graphslam->setCurrentIndex(instance_settings.value("graphslam").toInt());
  if(instance_settings.contains("loop_size"))
    this->ui_.sb_loop_size->setValue(instance_settings.value("loop_size").toInt());
  if(instance_settings.contains("cl_max_distance"))
    this->ui_.sb_cl_max_distance->setValue(instance_settings.value("cl_max_distance").toInt());
  if(instance_settings.contains("cl_p2p_distance"))
    this->ui_.dsb_cl_p2p_distance->setValue(instance_settings.value("cl_p2p_distance").toDouble());
  if(instance_settings.contains("cl_iterations"))
    this->ui_.sb_cl_iterations->setValue(instance_settings.value("cl_iterations").toInt());
  if(instance_settings.contains("slam_iterations"))
    this->ui_.sb_slam_iterations->setValue(instance_settings.value("slam_iterations").toInt());
  if(instance_settings.contains("slam_epsilon"))
    this->ui_.dsb_graph_epsilon->setValue(instance_settings.value("slam_epsilon").toDouble());
  if(instance_settings.contains("slam_p2p_distance"))
    this->ui_.dsb_graph_p2p_distance->setValue(instance_settings.value("slam_p2p_distance").toDouble());

  this->ui_.tb_settings->setCurrentIndex(0);
}

// work
void GuiPlugin::on_pb_start_pressed() {

  this->ui_.pb_start->setEnabled(false);
  this->ui_.pb_cancel->setVisible(true);
  this->ui_.pb_progress->setVisible(true);
  this->ui_.pb_progress->setRange(0, 0);

  this->mapping->start_mapping();
}

void GuiPlugin::on_work_finished(int exit_code) {
  this->ui_.pb_start->setEnabled(true);
  this->ui_.pb_progress->setVisible(false);
  this->ui_.pb_cancel->setVisible(false);

  if(exit_code == 1) {
    ROS_ERROR("Something went wrong");
  } else {
    std::vector<double> icp_results = this->mapping->get_icp_results();
    
    if(icp_results.size() == 0) {
      return;
    }

    this->ui_.tw_results->setEnabled(true);
    this->ui_.tw_results->clear();
    this->ui_.tw_results->setRowCount(0);
    this->ui_.tw_results->verticalHeader()->setVisible(false);

    this->ui_.tw_results->setHorizontalHeaderItem(0, new QTableWidgetItem("from"));
    this->ui_.tw_results->setHorizontalHeaderItem(1, new QTableWidgetItem("to"));
    this->ui_.tw_results->setHorizontalHeaderItem(2, new QTableWidgetItem("error"));

    for(int i = 0; i < icp_results.size(); i++) {
      int row = this->ui_.tw_results->rowCount();
      this->ui_.tw_results->insertRow(row);

      QTableWidgetItem* item = new QTableWidgetItem(QString::number(this->ui_.sb_first->value() + row));
      this->ui_.tw_results->setItem(row, 0, item);

      item = new QTableWidgetItem(QString::number(this->ui_.sb_first->value() + row + 1));
      this->ui_.tw_results->setItem(row, 1, item);

      item = new QTableWidgetItem(QString::number(icp_results[i]));
      this->ui_.tw_results->setItem(row, 2, item);
    }

    this->ui_.tb_settings->setVisible(false);
    this->ui_.tw_results->setVisible(true);
    this->ui_.pb_back->setVisible(true);
  }
}

void GuiPlugin::on_rosbag_finished() {
  this->ui_.sb_total->setValue(100);
}

void GuiPlugin::on_pb_show_pressed() {
  this->mapping->showResults();
  //this->mapping->calculate_crispnesses(7, 8);
  //this->mapping->segmentPointCloud();
}

void GuiPlugin::on_pb_cancel_pressed() {
  int ret = QMessageBox::warning(this->widget_, "Cancel", "Are you sure you want to caancel?", QMessageBox::Ok, QMessageBox::No);

  if( ret == QMessageBox::Ok) {
    this->mapping->cancel_mapping();
  }
}

void GuiPlugin::on_cb_update_scans_state_changed(int state) {
  if(state == 0) {
    this->ui_.tb_settings->removeTab(2);
    this->ui_.tb_settings->removeTab(1);
    this->mapping->set_use_rosbag(false);
  }
  else if(state == 2) {
    QWidget* tab_topics = this->ui_.tb_settings->findChild<QWidget*>("tab_topics");
    this->ui_.tb_settings->insertTab(1, tab_topics, "Topics");

    QWidget* tab_rosbag = this->ui_.tb_settings->findChild<QWidget*>("tab_rosbag");
    this->ui_.tb_settings->insertTab(1, tab_rosbag, "Rosbag");
    this->mapping->set_use_rosbag(true);
  }
}

void GuiPlugin::on_pb_back_pressed() {
  this->ui_.tw_results->setVisible(false);
  this->ui_.tb_settings->setVisible(true);
  this->ui_.pb_back->setVisible(false);
}

// init
void GuiPlugin::initWidgets() {
  this->initComboBoxes();
  this->ui_.pb_progress->setVisible(false);
  this->ui_.pb_cancel->setVisible(false);
  this->ui_.tw_results->setVisible(false);
  this->ui_.pb_back->setVisible(false);
}

void GuiPlugin::initComboBoxes() {
  this->ui_.cb_icp_minimization->addItem("Unit Quaternion", QVariant(1)); 
  this->ui_.cb_icp_minimization->addItem("Singular Value Decomposition", QVariant(2));
  //this->ui_.cb_minimization->addItem("Orthonormal Matrices");
  //this->ui_.cb_minimization->addItem("Dual Quaternions");
  //this->ui_.cb_minimization->addItem("Helix Approximation");
  this->ui_.cb_icp_minimization->addItem("Small Angle Approximation", QVariant(6));
  //this->ui_.cb_minimization->addItem("Uncertainty Based: Euler Angles");
  //this->ui_.cb_minimization->addItem("Uncertainty Based: Quaternions"); 
  //this->ui_.cb_minimization->addItem("Unit Quaternion with Scale Method");
                    
  this->ui_.cb_nn->addItem("simple k-d tree", QVariant(0)); 
  this->ui_.cb_nn->addItem("cached k-d tree", QVariant(1));
  //this->ui_.cb_nn->addItem("ANN tree");
  //this->ui_.cb_nn->addItem("BOC tree");

  this->ui_.cb_closing_loop->addItem("no loop closing", QVariant(0));
  //this->ui_.cb_closing_loop->addItem("Euler Angles", QVariant(1));
  this->ui_.cb_closing_loop->addItem("Quaternions", QVariant(2));
  this->ui_.cb_closing_loop->addItem("Unit Quaternions", QVariant(3));
  this->ui_.cb_closing_loop->addItem("SLERP", QVariant(4));

  this->ui_.cb_graphslam->addItem("no GraphSLAM", QVariant(0));
  this->ui_.cb_graphslam->addItem("Euler Angles", QVariant(1));
  this->ui_.cb_graphslam->addItem("Unit Quaternions", QVariant(2));
  this->ui_.cb_graphslam->addItem("Helix Approximation", QVariant(3));
  this->ui_.cb_graphslam->addItem("Small Angle Approximation", QVariant(4));
}

// callbacks
void GuiPlugin::scan_to_file_count_callback(const std_msgs::Int32::ConstPtr& count) {
  //this->mapping->set_file_count(count->data);
}

// work stuff
int GuiPlugin::runCommand(std::string command) {
  return system(command.c_str());
}

void GuiPlugin::shutdownPlugin() 
{
  this->n.shutdown();
  // unregister all publishers here
  this->mapping->cancel_mapping();
}

void GuiPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
   // rosbag
  instance_settings.setValue("rosbag_filename", this->ui_.le_filePath->text());
  instance_settings.setValue("input_is_meter", this->ui_.rb_meter->isChecked());
  instance_settings.setValue("input_is_lefthanded", this->ui_.rb_lefthanded->isChecked());
  // topics
  instance_settings.setValue("scan_topic", this->ui_.le_scan->text());
  instance_settings.setValue("odom_topic", this->ui_.le_odom->text());
  instance_settings.setValue("gps_topic", this->ui_.le_gps->text());
  // output
  instance_settings.setValue("output_filepath", this->ui_.le_output->text());
  // general
  instance_settings.setValue("total", this->ui_.sb_total->value());
  instance_settings.setValue("first_scan", this->ui_.sb_first->value());
  instance_settings.setValue("last_scan", this->ui_.sb_last->value());
  instance_settings.setValue("min_distance", this->ui_.dsb_min->value());
  instance_settings.setValue("max_distance", this->ui_.dsb_max->value());
  instance_settings.setValue("correspondances", this->ui_.cb_correspondances->currentIndex());
  instance_settings.setValue("metascan", this->ui_.cb_metascan->isChecked());
  instance_settings.setValue("export", this->ui_.cb_export->isChecked());
  instance_settings.setValue("export_path", this->ui_.le_export->text());
  // ICP
  instance_settings.setValue("icp_minimization", this->ui_.cb_icp_minimization->currentIndex());
  instance_settings.setValue("nearest_neighbor", this->ui_.cb_nn->currentIndex());
  instance_settings.setValue("icp_iterations", this->ui_.sb_icp_iterations->value());
  instance_settings.setValue("icp_epsilon", this->ui_.dsb_icp_epsilon->value());
  instance_settings.setValue("nn_max_p2p_distance", this->ui_.dsb_nn_p2p_distance->value());
  // GraphSLAM
  instance_settings.setValue("closing_loop", this->ui_.cb_closing_loop->currentIndex());
  instance_settings.setValue("graphslam", this->ui_.cb_graphslam->currentIndex());
  instance_settings.setValue("loop_size", this->ui_.sb_loop_size->value());
  instance_settings.setValue("cl_max_distance", this->ui_.sb_cl_max_distance->value());
  instance_settings.setValue("cl_p2p_distance", this->ui_.dsb_cl_p2p_distance->value());
  instance_settings.setValue("cl_iterations", this->ui_.sb_cl_iterations->value());
  instance_settings.setValue("slam_iterations", this->ui_.sb_slam_iterations->value());
  instance_settings.setValue("slam_epsilon", this->ui_.dsb_graph_epsilon->value());
  instance_settings.setValue("slam_p2p_distance", this->ui_.dsb_graph_p2p_distance->value());
  // work
  instance_settings.setValue("update_scans_state", this->ui_.cb_update_scans->isChecked());
  instance_settings.setValue("active_tab", this->ui_.tb_settings->currentIndex());
}

void GuiPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
 {
  // rosbag
  if(instance_settings.contains("rosbag_filename"))
    this->ui_.le_filePath->setText(instance_settings.value("rosbag_filename").toString());
  if(instance_settings.contains("input_is_meter"))
    this->ui_.rb_meter->setChecked(instance_settings.value("input_is_meter").toBool());
  if(instance_settings.contains("input_is_lefthanded"))
    this->ui_.rb_lefthanded->setChecked(instance_settings.value("input_is_lefthanded").toBool());
  // topics
  if(instance_settings.contains("scan_topic"))
    this->ui_.le_scan->setText(instance_settings.value("scan_topic").toString());
  if(instance_settings.contains("odom_topic"))
    this->ui_.le_odom->setText(instance_settings.value("odom_topic").toString());
  if(instance_settings.contains("gps_topic"))
    this->ui_.le_gps->setText(instance_settings.value("gps_topic").toString());
  // output
  if(instance_settings.contains("output_filepath"))
    this->ui_.le_output->setText(instance_settings.value("output_filepath").toString());
  // general
  if(instance_settings.contains("total"))
    this->ui_.sb_total->setValue(instance_settings.value("total").toInt());
  if(instance_settings.contains("first_scan"))
    this->ui_.sb_first->setValue(instance_settings.value("first_scan").toInt());
  if(instance_settings.contains("last_scan"))
    this->ui_.sb_last->setValue(instance_settings.value("last_scan").toInt());
  if(instance_settings.contains("min_distance"))
    this->ui_.dsb_min->setValue(instance_settings.value("min_distance").toDouble());
  if(instance_settings.contains("max_distance"))
    this->ui_.dsb_max->setValue(instance_settings.value("max_distance").toDouble());
  if(instance_settings.contains("correspondances"))
    this->ui_.cb_correspondances->setCurrentIndex(instance_settings.value("correspondances").toInt());
  if(instance_settings.contains("metascan"))
    this->ui_.cb_metascan->setChecked(instance_settings.value("metascan").toBool());
  if(instance_settings.contains("export"))
    this->ui_.cb_export->setChecked(instance_settings.value("export").toBool());
  if(instance_settings.contains("export_path"))
    this->ui_.le_export->setText(instance_settings.value("export_path").toString());
  // ICP
  if(instance_settings.contains("icp_minimization"))
    this->ui_.cb_icp_minimization->setCurrentIndex(instance_settings.value("icp_minimization").toInt());
  if(instance_settings.contains("nearest_neighbor"))
    this->ui_.cb_nn->setCurrentIndex(instance_settings.value("nearest_neighbor").toInt());
  if(instance_settings.contains("icp_iterations"))
    this->ui_.sb_icp_iterations->setValue(instance_settings.value("icp_iterations").toInt());
  if(instance_settings.contains("icp_epsilon"))
    this->ui_.dsb_icp_epsilon->setValue(instance_settings.value("icp_epsilon").toDouble());
  if(instance_settings.contains("nn_max_p2p_distance"))
    this->ui_.dsb_nn_p2p_distance->setValue(instance_settings.value("nn_max_p2p_distance").toDouble());
  // GraphSLAM
  if(instance_settings.contains("closing_loop"))
    this->ui_.cb_closing_loop->setCurrentIndex(instance_settings.value("closing_loop").toInt());
  if(instance_settings.contains("graphslam"))
    this->ui_.cb_graphslam->setCurrentIndex(instance_settings.value("graphslam").toInt());
  if(instance_settings.contains("loop_size"))
    this->ui_.sb_loop_size->setValue(instance_settings.value("loop_size").toInt());
  if(instance_settings.contains("cl_max_distance"))
    this->ui_.sb_cl_max_distance->setValue(instance_settings.value("cl_max_distance").toInt());
  if(instance_settings.contains("cl_p2p_distance"))
    this->ui_.dsb_cl_p2p_distance->setValue(instance_settings.value("cl_p2p_distance").toDouble());
  if(instance_settings.contains("cl_iterations"))
    this->ui_.sb_cl_iterations->setValue(instance_settings.value("cl_iterations").toInt());
  if(instance_settings.contains("slam_iterations"))
    this->ui_.sb_slam_iterations->setValue(instance_settings.value("slam_iterations").toInt());
  if(instance_settings.contains("slam_epsilon"))
    this->ui_.dsb_graph_epsilon->setValue(instance_settings.value("slam_epsilon").toDouble());
  if(instance_settings.contains("slam_p2p_distance"))
    this->ui_.dsb_graph_p2p_distance->setValue(instance_settings.value("slam_p2p_distance").toDouble());
  // work
  if(instance_settings.contains("update_scans_state"))
    this->ui_.cb_update_scans->setChecked(instance_settings.value("update_scans_state").toBool());
  if(instance_settings.contains("active_tab"))
    this->ui_.tb_settings->setCurrentIndex(instance_settings.value("active_tab").toInt());
}

} 
PLUGINLIB_DECLARE_CLASS(grabe_mapping, GuiPlugin, grabe_mapping::GuiPlugin, rqt_gui_cpp::Plugin)
