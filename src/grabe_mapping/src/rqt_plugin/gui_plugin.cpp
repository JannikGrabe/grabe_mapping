
#include "rqt_plugin/gui_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QtConcurrent/QtConcurrent>
#include <QStringList>
#include <QFileDialog>
#include <QLineEdit>
#include <QProgressBar>
#include <QMessageBox>

namespace grabe_mapping
{

GuiPlugin::GuiPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("GuiPlugin");

  this->count_sub = this->n.subscribe("mapping/scan_to_file_count", 10, &GuiPlugin::scan_to_file_count_callback, this);

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

  this->mapping = new Mapping();

  this->initWidgets();
  
  context.addWidget(widget_);

  // rosbag
  QObject::connect(ui_.cb_use_output_files, &QCheckBox::stateChanged, this, &GuiPlugin::on_cb_use_output_files_state_changed);
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
  QObject::connect(ui_.pb_start, &QPushButton::pressed, this, &GuiPlugin::on_pb_start_pressed);
  QObject::connect(ui_.pb_show, &QPushButton::pressed, this, &GuiPlugin::on_pb_show_pressed);

  // general
  QObject::connect(this->ui_.sb_first, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &GuiPlugin::on_sb_first_value_changed);
  QObject::connect(this->ui_.sb_last, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &GuiPlugin::on_sb_last_value_changed);
  QObject::connect(this->ui_.dsb_min, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &GuiPlugin::on_dsb_min_value_changed);
  QObject::connect(this->ui_.dsb_max, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &GuiPlugin::on_dsb_max_value_changed);
  QObject::connect(this->ui_.cb_correspondances, &QComboBox::currentTextChanged, this, &GuiPlugin::on_cb_correspondances_current_text_changed);
  QObject::connect(this->ui_.cb_metascan, &QCheckBox::stateChanged, this, &GuiPlugin::on_cb_metascan_state_changed);
  QObject::connect(this->ui_.cb_export, &QCheckBox::stateChanged, this, &GuiPlugin::on_cb_export_state_changed);

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
  QObject::connect(this->ui_.sb_cl_min_overlap, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &GuiPlugin::on_sb_cl_min_overlap_value_changed); 
  QObject::connect(this->ui_.dsb_cl_p2p_distance, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &GuiPlugin::on_dsb_cl_p2p_distance_value_changed);
  QObject::connect(this->ui_.sb_cl_iterations, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &GuiPlugin::on_sb_cl_iterations_value_changed); 
  QObject::connect(this->ui_.sb_slam_iterations, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &GuiPlugin::on_sb_slam_iterations_value_changed); 
  QObject::connect(this->ui_.dsb_graph_epsilon, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &GuiPlugin::on_dsb_graph_epsilon_value_changed);
  QObject::connect(this->ui_.dsb_graph_p2p_distance, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &GuiPlugin::on_dsb_graph_p2p_distance_value_changed);
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
void GuiPlugin::on_cb_use_output_files_state_changed(int state) {
  if(state == 2) {
    this->ui_.tb_settings->removeTab(2);
    this->ui_.tb_settings->removeTab(1);
    this->mapping->set_use_rosbag(false);
  }
  else if(state == 0) {
    QWidget* tab_topics = this->ui_.tb_settings->findChild<QWidget*>("tab_topics");
    this->ui_.tb_settings->insertTab(1, tab_topics, "Topics");

    QWidget* tab_rosbag = this->ui_.tb_settings->findChild<QWidget*>("tab_rosbag");
    this->ui_.tb_settings->insertTab(1, tab_rosbag, "Rosbag");
    this->mapping->set_use_rosbag(true);
  }
}

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
void GuiPlugin::on_sb_first_value_changed(int val) {
  
  int last = this->ui_.sb_last->value();

  if(val == -1) { // set inactive
    this->mapping->set_parameter_active("-s", false);
  } else {
    this->mapping->set_parameter_active("-s", true);
    this->mapping->set_parameter_value("-s", val);

    if(last != -1 && val >= last) // first is larger than last
      this->ui_.sb_last->setValue(val + 1);
  }
}

void GuiPlugin::on_sb_last_value_changed(int val) {
  static int previous = -1;
  int first = this->ui_.sb_first->value();

  if(val == -1) { // set inactive
    this->mapping->set_parameter_active("-e", false);
    previous = -1;
  } else if(val == 0 && previous != -1) { // skip 0 to -1
    this->ui_.sb_last->setValue(-1);
  } else if (val == 0 && previous == -1) { // skip 0 to 1
    this->ui_.sb_last->setValue(1);
  } else {
    this->mapping->set_parameter_active("-e", true);
    this->mapping->set_parameter_value("-e", val);

    if(first != -1 && val <= first) { // last is smaller than first
      if(previous == -1) {            // was inactive -> set to one higher than first
        this->ui_.sb_last->setValue(first + 1);
        return;
      } else {                        // wasn't inactive -> set first one lower than this
        this->ui_.sb_first->setValue(val - 1);
      }
  }
    previous = val;
  }
}

void GuiPlugin::on_dsb_min_value_changed(double val) {
  double max = this->ui_.dsb_max->value();
  
  if(val < 0.0) { // set inactive
    this->mapping->set_parameter_active("--min", false);
  } else {
    this->mapping->set_parameter_active("--min", true);
    this->mapping->set_parameter_value("--min", val);

    if(max >= 0.0 && val >= max) // first is larger than last
      this->ui_.dsb_max->setValue(val + 1.0);
  }
}

void GuiPlugin::on_dsb_max_value_changed(double val) {
  static double previous = -1;
  double min = this->ui_.dsb_min->value();

  if(val < 0.0) { // set inactive
    this->mapping->set_parameter_active("--max", false);
    previous = val;
  } else {
    this->mapping->set_parameter_active("--max", true);
    this->mapping->set_parameter_value("--max", val);

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
    this->mapping->set_parameter_active("--normal-shoot-simple", false);
    this->mapping->set_parameter_active("--point-to-plane-simple", false);
  } else if(text == "closest along normal") {
    this->mapping->set_parameter_active("--normal-shoot-simple", true);
    this->mapping->set_parameter_active("--point-to-plane-simple", false);
  } else if(text == "closest point-to-plane distance") {
    this->mapping->set_parameter_active("--normal-shoot-simple", false);
    this->mapping->set_parameter_active("--point-to-plane-simple", true);
  }
}

void GuiPlugin::on_cb_metascan_state_changed(int state) {
  if(state) {
    this->mapping->set_parameter_active("--metascan", true);
  } else {
    this->mapping->set_parameter_active("--metascan", false);
  }
}

void GuiPlugin::on_cb_export_state_changed(int state) {
  if(state) {
    this->mapping->set_parameter_active("--exportAllPoints", true);
    this->ui_.le_export->setEnabled(true);
    this->ui_.pb_export->setEnabled(true);
  } else {
    this->mapping->set_parameter_active("--exportAllPoints", false);
    this->ui_.le_export->setEnabled(false);
    this->ui_.pb_export->setEnabled(false);
  }
}

void GuiPlugin::on_pb_export_pressed() {

}

// ICP
void GuiPlugin::on_cb_icp_minimization_current_text_changed(QString text) {

  int index = this->ui_.cb_icp_minimization->findText(text);
  int param_value = this->ui_.cb_icp_minimization->itemData(index).toInt();

  if(!this->mapping->set_parameter_value("-a", param_value)) {
    QMessageBox::critical(this->widget_, "Warning", "Could not find " + text, QMessageBox::Ok);
  }
}

void GuiPlugin::on_cb_nn_current_text_changed(QString text) {
  int index = this->ui_.cb_nn->findText(text);
  int param_value = this->ui_.cb_nn->itemData(index).toInt();

  if(!this->mapping->set_parameter_value("-t", param_value)) {
    QMessageBox::critical(this->widget_, "Warning", "Could not find " + text, QMessageBox::Ok);
  }
}

void GuiPlugin::on_sb_icp_iterations_value_changed(int val) {

  this->mapping->set_parameter_value("-i", val);
}

void GuiPlugin::on_dsb_icp_epsilon_value_changed(double val) {
    this->mapping->set_parameter_value("--epsICP", val);

}

void GuiPlugin::on_dsb_nn_p2p_distance_value_changed(double val) {
    this->mapping->set_parameter_value("--dist", val);
}

// GraphSLAM
void GuiPlugin::on_cb_closing_loop_current_text_changed(QString text) {
  int index = this->ui_.cb_closing_loop->findText(text);
  int param_value = this->ui_.cb_closing_loop->itemData(index).toInt();

  if(!this->mapping->set_parameter_value("-L", param_value)) {
    QMessageBox::critical(this->widget_, "Warning", "Could not find " + text, QMessageBox::Ok);
  }
}

void GuiPlugin::on_cb_graphslam_current_text_changed(QString text) {
  int index = this->ui_.cb_graphslam->findText(text);
  int param_value = this->ui_.cb_graphslam->itemData(index).toInt();

  if(!this->mapping->set_parameter_value("-G", param_value)) {
    QMessageBox::critical(this->widget_, "Warning", "Could not find " + text, QMessageBox::Ok);
  } 
}

void GuiPlugin::on_sb_loop_size_value_changed(int val) {
  this->mapping->set_parameter_value("--loopsize", val);
}

void GuiPlugin::on_sb_cl_max_distance_value_changed(int val) {
  this->mapping->set_parameter_value("--cldist", val);
}

void GuiPlugin::on_sb_cl_min_overlap_value_changed(int val) {
  this->mapping->set_parameter_value("--clpairs", val);
}

void GuiPlugin::on_dsb_cl_p2p_distance_value_changed(double val) {
  this->mapping->set_parameter_value("--distLoop", val);
}

void GuiPlugin::on_sb_cl_iterations_value_changed(int val) {
  this->mapping->set_parameter_value("--iterLoop", val);
}

void GuiPlugin::on_sb_slam_iterations_value_changed(int val) {
  this->mapping->set_parameter_value("-I", val);
}

void GuiPlugin::on_dsb_graph_epsilon_value_changed(double val) {
  this->mapping->set_parameter_value("--epsSLAM", val);
}

void GuiPlugin::on_dsb_graph_p2p_distance_value_changed(double val) {
  this->mapping->set_parameter_value("--distSLAM", val);
}

// work
void GuiPlugin::on_pb_start_pressed() {

  this->ui_.pb_start->setEnabled(false);
  this->ui_.pb_show->setEnabled(false);
  this->ui_.pb_progress->setVisible(true);
  this->ui_.pb_progress->setRange(0, 0);

  this->mapping->start_mapping();
}

void GuiPlugin::on_work_finished(int exit_code) {
  this->ui_.pb_start->setEnabled(true);
  this->ui_.pb_progress->setVisible(false);
  this->ui_.pb_show->setEnabled(true);

  if(exit_code == 1) {
    ROS_ERROR("Something went wrong");
  }
}

void GuiPlugin::on_pb_show_pressed() {
  this->mapping->showResults();
  this->ui_.pb_show->setEnabled(false);
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
  this->mapping->set_output_filepath(text);
}

// init
void GuiPlugin::initWidgets() {
  this->initComboBoxes();
  this->ui_.pb_progress->setVisible(false);
  this->ui_.pb_show->setEnabled(false);
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
  this->ui_.cb_closing_loop->addItem("Euler Angles", QVariant(1));
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
  this->mapping->set_file_count(count->data);
}

// work stuff
int GuiPlugin::runCommand(std::string command) {
  return system(command.c_str());
}

void GuiPlugin::shutdownPlugin()
{
  // unregister all publishers here
  this->mapping->cancel_mapping();
}

void GuiPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
  // rosbag
  instance_settings.setValue("rosbag_filename", this->mapping->get_rosbag_filename());
  instance_settings.setValue("input_is_meter", this->mapping->get_input_is_meter());
  instance_settings.setValue("input_is_lefthanded", this->mapping->get_input_is_lefthanded());
  // topics
  instance_settings.setValue("scan_topic", this->mapping->get_scan_topic());
  instance_settings.setValue("odom_topic", this->mapping->get_odom_topic());
  instance_settings.setValue("gps_topic", this->mapping->get_gps_topic());
  // output
  instance_settings.setValue("output_filepath", this->mapping->get_output_filepath());
  instance_settings.setValue("use_output_files", !this->mapping->get_use_rosbag());
  // general
  instance_settings.setValue("total", this->ui_.le_total->text());
  instance_settings.setValue("first_scan", this->ui_.sb_first->value());
  instance_settings.setValue("last_scan", this->ui_.sb_last->value());
  instance_settings.setValue("min_distance", this->ui_.dsb_min->value());
  instance_settings.setValue("max_distance", this->ui_.dsb_max->value());
  instance_settings.setValue("correspondances", this->ui_.cb_correspondances->currentIndex());
  instance_settings.setValue("metascan", this->ui_.cb_metascan->isChecked());
  instance_settings.setValue("export", this->ui_.cb_export->isChecked());
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
  instance_settings.setValue("cl_min_overlap", this->ui_.sb_cl_min_overlap->value());
  instance_settings.setValue("cl_p2p_distance", this->ui_.dsb_cl_p2p_distance->value());
  instance_settings.setValue("cl_iterations", this->ui_.sb_cl_iterations->value());
  instance_settings.setValue("slam_iterations", this->ui_.sb_slam_iterations->value());
  instance_settings.setValue("slam_epsilon", this->ui_.dsb_graph_epsilon->value());
  instance_settings.setValue("slam_p2p_distance", this->ui_.dsb_graph_p2p_distance->value());
}

void GuiPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
  // rosbag
  this->ui_.le_filePath->setText(instance_settings.value("rosbag_filename").toString());
  this->ui_.rb_meter->setChecked(instance_settings.value("input_is_meter").toBool());
  this->ui_.rb_lefthanded->setChecked(instance_settings.value("input_is_lefthanded").toBool());
  // topics
  this->ui_.le_scan->setText(instance_settings.value("scan_topic").toString());
  this->ui_.le_odom->setText(instance_settings.value("odom_topic").toString());
  this->ui_.le_gps->setText(instance_settings.value("gps_topic").toString());
  // output
  this->ui_.le_output->setText(instance_settings.value("output_filepath").toString());
  this->ui_.cb_use_output_files->setChecked(instance_settings.value("use_output_files").toBool());
  // general
  this->ui_.le_total->setText(instance_settings.value("total").toString());
  this->ui_.sb_first->setValue(instance_settings.value("first_scan").toInt());
  this->ui_.sb_last->setValue(instance_settings.value("last_scan").toInt());
  this->ui_.dsb_min->setValue(instance_settings.value("min_distance").toDouble());
  this->ui_.dsb_max->setValue(instance_settings.value("max_distance").toDouble());
  this->ui_.cb_correspondances->setCurrentIndex(instance_settings.value("correspondances").toInt());
  this->ui_.cb_metascan->setChecked(instance_settings.value("metascan").toBool());
  this->ui_.cb_export->setChecked(instance_settings.value("export").toBool());
  // ICP
  this->ui_.cb_icp_minimization->setCurrentIndex(instance_settings.value("icp_minimization").toInt());
  this->ui_.cb_nn->setCurrentIndex(instance_settings.value("nearest_neighbor").toInt());
  this->ui_.sb_icp_iterations->setValue(instance_settings.value("icp_iterations").toInt());
  this->ui_.dsb_icp_epsilon->setValue(instance_settings.value("icp_epsilon").toDouble());
  this->ui_.dsb_nn_p2p_distance->setValue(instance_settings.value("nn_max_p2p_distance").toDouble());
  // GraphSLAM
  this->ui_.cb_closing_loop->setCurrentIndex(instance_settings.value("closing_loop").toInt());
  this->ui_.cb_graphslam->setCurrentIndex(instance_settings.value("graphslam").toInt());
  this->ui_.sb_loop_size->setValue(instance_settings.value("loop_size").toInt());
  this->ui_.sb_cl_max_distance->setValue(instance_settings.value("cl_max_distance").toInt());
  this->ui_.sb_cl_min_overlap->setValue(instance_settings.value("cl_min_overlap").toInt());
  this->ui_.dsb_cl_p2p_distance->setValue(instance_settings.value("cl_p2p_distance").toDouble());
  this->ui_.sb_cl_iterations->setValue(instance_settings.value("cl_iterations").toInt());
  this->ui_.sb_slam_iterations->setValue(instance_settings.value("slam_iterations").toInt());
  this->ui_.dsb_graph_epsilon->setValue(instance_settings.value("slam_epsilon").toDouble());
  this->ui_.dsb_graph_p2p_distance->setValue(instance_settings.value("slam_p2p_distance").toDouble());
}

} 
PLUGINLIB_DECLARE_CLASS(grabe_mapping, GuiPlugin, grabe_mapping::GuiPlugin, rqt_gui_cpp::Plugin)
