
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

  this->mapping_manager = new Mapping_manager();
  
  Mapping* mapping = new Mapping();
  this->mapping_manager->addMapping(mapping);

  this->rosbag_reader = new Rosbag_reader();

  this->initWidgets();

  this->stfw = new Scan_to_file_widget(this->rosbag_reader, "Read Scans from Rosbag", this->widget_);
  this->ui_.vl_main->insertWidget(0, stfw); 

  Parameter_widget* pw = new Parameter_widget(mapping, "SLAM Parameters");
  this->ui_.hl_main->insertWidget(2, pw);
  this->pws.push_back(pw);

  context.addWidget(widget_);

  // work
  QObject::connect(this->mapping_manager, &Mapping_manager::finished_mapping, this, &GuiPlugin::on_mapping_finished);
  QObject::connect(ui_.pb_start, &QPushButton::pressed, this, &GuiPlugin::on_pb_start_pressed);
  QObject::connect(ui_.pb_show, &QPushButton::pressed, this, &GuiPlugin::on_pb_show_pressed);
  QObject::connect(this->ui_.pb_back, &QPushButton::pressed, this, &GuiPlugin::on_pb_back_pressed);

  // export
  QObject::connect(this->ui_.cb_export, &QCheckBox::stateChanged, this, &GuiPlugin::on_cb_export_state_changed);
  QObject::connect(this->ui_.le_export, &QLineEdit::textChanged, this, &GuiPlugin::on_le_export_text_changed);
  QObject::connect(this->ui_.pb_export, &QPushButton::pressed, this, &GuiPlugin::on_pb_export_pressed);
  QObject::connect(this->ui_.pb_export_save, &QPushButton::pressed, this, &GuiPlugin::on_pb_export_save_pressed);
  
  // source dir
  QObject::connect(this->ui_.pb_source_dir, &QPushButton::pressed, this, &GuiPlugin::on_pb_source_dir_pressed);
  QObject::connect(this->ui_.le_source_dir, &QLineEdit::textChanged, this, &GuiPlugin::on_le_source_dir_text_changed);
  QObject::connect(this->rosbag_reader, &Rosbag_reader::output_path_changed, this->ui_.le_source_dir, &QLineEdit::setText);

  // config
  //QObject::connect(this->ui_.pb_save_config, &QPushButton::pressed, this, &GuiPlugin::on_pb_save_config_pressed);
  //QObject::connect(this->ui_.pb_load_config, &QPushButton::pressed, this, &GuiPlugin::on_pb_load_config_pressed);
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
// general
void GuiPlugin::on_cb_export_state_changed(int state) {
  //this->mapping->set_export_pts(state);
  this->ui_.le_export->setEnabled(state);
  this->ui_.pb_export->setEnabled(state);
  this->ui_.pb_export_save->setEnabled(state);
}

void GuiPlugin::on_pb_export_pressed() {
  QString filename = QFileDialog::getSaveFileName(
        this->widget_,
        "export file", 
        this->ui_.le_source_dir->text() +  "/points.pts",
        tr("export file (*.pts)"));

  this->ui_.le_export->setText(filename);
}

void GuiPlugin::on_le_export_text_changed(QString text) {
  //this->mapping->set_export_path(text);
}

void GuiPlugin::on_pb_export_save_pressed() {
  std::cout << "not yet implemented" << std::endl;
}

void GuiPlugin::on_pb_source_dir_pressed() {
  QString path = QFileDialog::getExistingDirectory(
        this->widget_,
        tr("Select Directory for output files"),
        "/home/jannik/Bachelorarbeit",
        QFileDialog::ShowDirsOnly
    );

    this->ui_.le_source_dir->setText(path + "/");
}

void GuiPlugin::on_le_source_dir_text_changed(QString text) {  
  this->mapping_manager->set_dir_path(text);

  QDir dir(text);
  QStringList files = dir.entryList(QStringList() << "*.3d", QDir::Files);

  if(files.size() > 0)
    this->pws[0]->set_total(files.size()); 
}

// config
void GuiPlugin::on_pb_save_config_pressed() {
  // QString filename = QFileDialog::getSaveFileName(this->widget_, "save config", "config.ini",
  //       tr("Config file (*.ini)"));

  // QSettings settings(filename, QSettings::IniFormat, this->widget_);

  // // rosbag
  // settings.setValue("rosbag_filename", this->ui_.le_filePath->text());
  // settings.setValue("input_is_meter", this->ui_.rb_meter->isChecked());
  // settings.setValue("input_is_lefthanded", this->ui_.rb_lefthanded->isChecked());
  // // topics
  // settings.setValue("scan_topic", this->ui_.le_scan->text());
  // settings.setValue("odom_topic", this->ui_.le_odom->text());
  // settings.setValue("gps_topic", this->ui_.le_gps->text());
  // // output
  // settings.setValue("output_filepath", this->ui_.le_output->text());
  // // general
  // settings.setValue("total", this->ui_.sb_total->value());
  // settings.setValue("first_scan", this->ui_.sb_first->value());
  // settings.setValue("last_scan", this->ui_.sb_last->value());
  // settings.setValue("min_distance", this->ui_.dsb_min->value());
  // settings.setValue("max_distance", this->ui_.dsb_max->value());
  // settings.setValue("correspondances", this->ui_.cb_correspondances->currentIndex());
  // settings.setValue("metascan", this->ui_.cb_metascan->isChecked());
  // settings.setValue("export", this->ui_.cb_export->isChecked());
  // settings.setValue("export_path", this->ui_.le_export->text());
  // // ICP
  // settings.setValue("icp_minimization", this->ui_.cb_icp_minimization->currentIndex());
  // settings.setValue("nearest_neighbor", this->ui_.cb_nn->currentIndex());
  // settings.setValue("icp_iterations", this->ui_.sb_icp_iterations->value());
  // settings.setValue("icp_epsilon", this->ui_.dsb_icp_epsilon->value());
  // settings.setValue("nn_max_p2p_distance", this->ui_.dsb_nn_p2p_distance->value());
  // // GraphSLAM
  // settings.setValue("closing_loop", this->ui_.cb_closing_loop->currentIndex());
  // settings.setValue("graphslam", this->ui_.cb_graphslam->currentIndex());
  // settings.setValue("loop_size", this->ui_.sb_loop_size->value());
  // settings.setValue("cl_max_distance", this->ui_.sb_cl_max_distance->value());
  // settings.setValue("cl_p2p_distance", this->ui_.dsb_cl_p2p_distance->value());
  // settings.setValue("cl_iterations", this->ui_.sb_cl_iterations->value());
  // settings.setValue("slam_iterations", this->ui_.sb_slam_iterations->value());
  // settings.setValue("slam_epsilon", this->ui_.dsb_graph_epsilon->value());
  // settings.setValue("slam_p2p_distance", this->ui_.dsb_graph_p2p_distance->value());
}

void GuiPlugin::on_pb_load_config_pressed() {
  // QString filename = QFileDialog::getOpenFileName(
  //   widget_,
  //   tr("Select Config File"),
  //   "",
  //   tr("*.ini")
  // );

  // QSettings instance_settings(filename, QSettings::IniFormat, this->widget_);
  
  // // rosbag
  // if(instance_settings.contains("rosbag_filename"))
  //   this->ui_.le_filePath->setText(instance_settings.value("rosbag_filename").toString());
  // if(instance_settings.contains("input_is_meter"))
  //   this->ui_.rb_meter->setChecked(instance_settings.value("input_is_meter").toBool());
  // if(instance_settings.contains("input_is_lefthanded"))
  //   this->ui_.rb_lefthanded->setChecked(instance_settings.value("input_is_lefthanded").toBool());
  // // topics
  // if(instance_settings.contains("scan_topic"))
  //   this->ui_.le_scan->setText(instance_settings.value("scan_topic").toString());
  // if(instance_settings.contains("odom_topic"))
  //   this->ui_.le_odom->setText(instance_settings.value("odom_topic").toString());
  // if(instance_settings.contains("gps_topic"))
  //   this->ui_.le_gps->setText(instance_settings.value("gps_topic").toString());
  // // output
  // if(instance_settings.contains("output_filepath"))
  //   this->ui_.le_output->setText(instance_settings.value("output_filepath").toString());
  // // general
  // if(instance_settings.contains("total"))
  //   this->ui_.sb_total->setValue(instance_settings.value("total").toInt());
  // if(instance_settings.contains("first_scan"))
  //   this->ui_.sb_first->setValue(instance_settings.value("first_scan").toInt());
  // if(instance_settings.contains("last_scan"))
  //   this->ui_.sb_last->setValue(instance_settings.value("last_scan").toInt());
  // if(instance_settings.contains("min_distance"))
  //   this->ui_.dsb_min->setValue(instance_settings.value("min_distance").toDouble());
  // if(instance_settings.contains("max_distance"))
  //   this->ui_.dsb_max->setValue(instance_settings.value("max_distance").toDouble());
  // if(instance_settings.contains("correspondances"))
  //   this->ui_.cb_correspondances->setCurrentIndex(instance_settings.value("correspondances").toInt());
  // if(instance_settings.contains("metascan"))
  //   this->ui_.cb_metascan->setChecked(instance_settings.value("metascan").toBool());
  // if(instance_settings.contains("export"))
  //   this->ui_.cb_export->setChecked(instance_settings.value("export").toBool());
  // if(instance_settings.contains("export_path"))
  //   this->ui_.le_export->setText(instance_settings.value("export_path").toString());
  // // ICP
  // if(instance_settings.contains("icp_minimization"))
  //   this->ui_.cb_icp_minimization->setCurrentIndex(instance_settings.value("icp_minimization").toInt());
  // if(instance_settings.contains("nearest_neighbor"))
  //   this->ui_.cb_nn->setCurrentIndex(instance_settings.value("nearest_neighbor").toInt());
  // if(instance_settings.contains("icp_iterations"))
  //   this->ui_.sb_icp_iterations->setValue(instance_settings.value("icp_iterations").toInt());
  // if(instance_settings.contains("icp_epsilon"))
  //   this->ui_.dsb_icp_epsilon->setValue(instance_settings.value("icp_epsilon").toDouble());
  // if(instance_settings.contains("nn_max_p2p_distance"))
  //   this->ui_.dsb_nn_p2p_distance->setValue(instance_settings.value("nn_max_p2p_distance").toDouble());
  // // GraphSLAM
  // if(instance_settings.contains("closing_loop"))
  //   this->ui_.cb_closing_loop->setCurrentIndex(instance_settings.value("closing_loop").toInt());
  // if(instance_settings.contains("graphslam"))
  //   this->ui_.cb_graphslam->setCurrentIndex(instance_settings.value("graphslam").toInt());
  // if(instance_settings.contains("loop_size"))
  //   this->ui_.sb_loop_size->setValue(instance_settings.value("loop_size").toInt());
  // if(instance_settings.contains("cl_max_distance"))
  //   this->ui_.sb_cl_max_distance->setValue(instance_settings.value("cl_max_distance").toInt());
  // if(instance_settings.contains("cl_p2p_distance"))
  //   this->ui_.dsb_cl_p2p_distance->setValue(instance_settings.value("cl_p2p_distance").toDouble());
  // if(instance_settings.contains("cl_iterations"))
  //   this->ui_.sb_cl_iterations->setValue(instance_settings.value("cl_iterations").toInt());
  // if(instance_settings.contains("slam_iterations"))
  //   this->ui_.sb_slam_iterations->setValue(instance_settings.value("slam_iterations").toInt());
  // if(instance_settings.contains("slam_epsilon"))
  //   this->ui_.dsb_graph_epsilon->setValue(instance_settings.value("slam_epsilon").toDouble());
  // if(instance_settings.contains("slam_p2p_distance"))
  //   this->ui_.dsb_graph_p2p_distance->setValue(instance_settings.value("slam_p2p_distance").toDouble());

  // this->ui_.tb_settings->setCurrentIndex(0);
}

// work
void GuiPlugin::on_pb_start_pressed() {

  this->ui_.pb_start->setEnabled(false);
  this->ui_.pb_progress->setVisible(true);
  this->ui_.pb_progress->setRange(0, 0);

  this->mapping_manager->start_mapping();
}

void GuiPlugin::on_mapping_finished(int exit_code) {
  this->ui_.pb_start->setEnabled(true);
  this->ui_.pb_progress->setVisible(false);

  if(exit_code != 0) {
    ROS_ERROR("Something went wrong");
    return;
  }

  this->mapping_manager->write_frames();

  std::vector<double> icp_results; // = this->mapping->get_icp_results();
  
  if(icp_results.size() != 0) {
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

      QTableWidgetItem* item = new QTableWidgetItem(QString::number(i));
      this->ui_.tw_results->setItem(row, 0, item);

      item = new QTableWidgetItem(QString::number(i + 1));
      this->ui_.tw_results->setItem(row, 1, item);

      item = new QTableWidgetItem(QString::number(icp_results[i]));
      this->ui_.tw_results->setItem(row, 2, item);
    }

    this->ui_.tw_results->setVisible(true);
    this->ui_.pb_back->setVisible(true);
  }

//create new instance of mapping

  Mapping* latest = this->mapping_manager->latest();
  
  Mapping* mapping = new Mapping(latest);
  this->mapping_manager->addMapping(mapping);

  Parameter_widget* pw = new Parameter_widget(mapping, "Improving SLAM");
  pw->set_total(Mapping_manager::last_scan - Mapping_manager::first_scan + 1);
  pw->set_start_min_max(Mapping_manager::first_scan, Mapping_manager::last_scan - 1);
  pw->set_end_min_max(Mapping_manager::first_scan + 1, Mapping_manager::last_scan);
  this->ui_.hl_main->insertWidget(2, pw);

  this->pws.back()->setVisible(false);
  this->pws.push_back(pw);
}

void GuiPlugin::on_pb_show_pressed() {
  this->mapping_manager->showResults();
}

void GuiPlugin::on_pb_back_pressed() {
  this->ui_.tw_results->setVisible(false);
  this->ui_.pb_back->setVisible(false);
}

// init
void GuiPlugin::initWidgets() {
  this->ui_.pb_progress->setVisible(false);
  this->ui_.tw_results->setVisible(false);
  this->ui_.pb_back->setVisible(false);
}

// callbacks
void GuiPlugin::scan_to_file_count_callback(const std_msgs::Int32::ConstPtr& count) {
  //this->mapping->set_file_count(count->data);
}

void GuiPlugin::shutdownPlugin() 
{
  this->n.shutdown();
  // unregister all publishers here
  //this->mapping->cancel_mapping();
}

void GuiPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
  this->pws[0]->save_settings(instance_settings);
  this->stfw->save_settings(instance_settings);

  instance_settings.setValue("export", this->ui_.cb_export->isChecked());
  instance_settings.setValue("export_path", this->ui_.le_export->text());
  instance_settings.setValue("source_dir", this->ui_.le_source_dir->text());
}

void GuiPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
 {
  if(instance_settings.contains("export"))
    this->ui_.cb_export->setChecked(instance_settings.value("export").toBool());
  if(instance_settings.contains("export_path"))
    this->ui_.le_export->setText(instance_settings.value("export_path").toString());
  if(instance_settings.contains("source_dir"))
    this->ui_.le_source_dir->setText(instance_settings.value("source_dir").toString());

  this->pws[0]->restore_settings(instance_settings);
  this->stfw->restore_settings(instance_settings);
}

} 
PLUGINLIB_DECLARE_CLASS(grabe_mapping, GuiPlugin, grabe_mapping::GuiPlugin, rqt_gui_cpp::Plugin)
