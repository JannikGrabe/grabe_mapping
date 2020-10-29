
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
  QObject::connect(ui_.pb_openrviz, &QPushButton::pressed, this, &GuiPlugin::on_pb_openrviz_pressed);

  // ICP
  QObject::connect(this->ui_.cb_minimization, &QComboBox::currentTextChanged, this, &GuiPlugin::on_cb_minimization_current_text_changed);
  QObject::connect(this->ui_.cb_nn, &QComboBox::currentTextChanged, this, &GuiPlugin::on_cb_nn_current_text_changed);

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
  }
  else if(state == 0) {
    QWidget* tab_topics = this->ui_.tb_settings->findChild<QWidget*>("tab_topics");
    this->ui_.tb_settings->insertTab(1, tab_topics, "Topics");

    QWidget* tab_rosbag = this->ui_.tb_settings->findChild<QWidget*>("tab_rosbag");
    this->ui_.tb_settings->insertTab(1, tab_rosbag, "Rosbag");
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

  // Algorithms
void GuiPlugin::on_cb_minimization_current_text_changed(QString text) {
  if(!this->mapping->set_minimization(text)) {
    QMessageBox::critical(this->widget_, "Warning", "Could not find " + text, QMessageBox::Ok);
    int i = this->ui_.cb_minimization->findText(text);
    if(i != -1) this->ui_.cb_minimization->removeItem(i);
  }
}

void GuiPlugin::on_cb_nn_current_text_changed(QString text) {
  if(!this->mapping->set_nearest_neighbor(text)) {
    QMessageBox::critical(this->widget_, "Warning", "Could not find " + text, QMessageBox::Ok);
    int i = this->ui_.cb_nn->findText(text);
    if(i != -1) this->ui_.cb_nn->removeItem(i);
  }
}

  // work
void GuiPlugin::on_pb_start_pressed() {

  this->ui_.pb_start->setEnabled(false);
  this->ui_.pb_progress->setVisible(true);
  this->ui_.pb_progress->setRange(0, 0);
  this->ui_.pb_openrviz->setVisible(true);

  this->mapping->start_mapping();
}

void GuiPlugin::on_work_finished(int exit_code) {
  this->ui_.pb_start->setEnabled(true);
  this->ui_.pb_progress->setVisible(false);
  this->ui_.pb_openrviz->setVisible(false);

  if(exit_code == 1) {
    ROS_ERROR("Something went wrong");
  }
}

void GuiPlugin::on_pb_openrviz_pressed() {
  QFuture<void> future = QtConcurrent::run(GuiPlugin::runCommand, std::string("rviz"));
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
  this->ui_.pb_openrviz->setVisible(false);
}

void GuiPlugin::initComboBoxes() {
  this->ui_.cb_minimization->addItem("Unit Quaternion"); 
  this->ui_.cb_minimization->addItem("Singular Value Decomposition");
  this->ui_.cb_minimization->addItem("Orthonormal Matrices");
  this->ui_.cb_minimization->addItem("Dual Quaternions");
  this->ui_.cb_minimization->addItem("Helix Approximation");
  this->ui_.cb_minimization->addItem("Small Angle Approximation");
  this->ui_.cb_minimization->addItem("Uncertainty Based: Euler Angles");
  this->ui_.cb_minimization->addItem("Uncertainty Based: Quaternions"); 
  this->ui_.cb_minimization->addItem("Unit Quaternion with Scale Method");
  this->ui_.cb_minimization->addItem("bla");
                    
  this->ui_.cb_nn->addItem("simple k-d tree"); 
  this->ui_.cb_nn->addItem("cached k-d tree");
  this->ui_.cb_nn->addItem("ANN tree");
  this->ui_.cb_nn->addItem("BOC tree");
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
}

void GuiPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
  instance_settings.setValue("rosbag_filename", this->mapping->get_rosbag_filename());
  instance_settings.setValue("input_is_meter", this->mapping->get_input_is_meter());
  instance_settings.setValue("input_is_lefthanded", this->mapping->get_input_is_lefthanded());
  instance_settings.setValue("scan_topic", this->mapping->get_scan_topic());
  instance_settings.setValue("odom_topic", this->mapping->get_odom_topic());
  instance_settings.setValue("gps_topic", this->mapping->get_gps_topic());
  instance_settings.setValue("output_filepath", this->mapping->get_output_filepath());
}

void GuiPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
  this->ui_.le_filePath->setText(instance_settings.value("rosbag_filename").toString());
  this->ui_.rb_meter->setChecked(instance_settings.value("input_is_meter").toBool());
  this->ui_.rb_lefthanded->setChecked(instance_settings.value("input_is_lefthanded").toBool());
  this->ui_.le_scan->setText(instance_settings.value("scan_topic").toString());
  this->ui_.le_odom->setText(instance_settings.value("odom_topic").toString());
  this->ui_.le_gps->setText(instance_settings.value("gps_topic").toString());
  this->ui_.le_output->setText(instance_settings.value("output_filepath").toString());
}

} 
PLUGINLIB_DECLARE_CLASS(grabe_mapping, GuiPlugin, grabe_mapping::GuiPlugin, rqt_gui_cpp::Plugin)
