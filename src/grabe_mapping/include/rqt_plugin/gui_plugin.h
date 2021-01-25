#ifndef GRABE_MAPPING_GUI_PLUGIN_H
#define GRABE_MAPPING_GUI_PLUGIN_H

#include <rqt_gui_cpp/plugin.h>
#include <grabe_mapping/ui_gui_plugin.h>
#include <QWidget>
#include <QMainWindow>
#include "mapping/mapping_manager.h"
#include "mapping/rosbag_reader.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "std_msgs/Int32.h"

#include "rqt_plugin/parameter_widget.h"
#include "rqt_plugin/scan_to_file_widget.h"

namespace grabe_mapping // the namespace determines the name by which the plugin can be discovered
{

class GuiPlugin
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  GuiPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
      qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();


public slots:

  // work
  void on_pb_start_pressed();
  void on_mapping_finished(int exit_code);
  void on_pb_show_pressed();
  void on_pb_back_pressed();

  // parameters
  void on_cb_export_state_changed(int state);
  void on_pb_export_pressed();
  void on_le_export_text_changed(QString text);
  void on_pb_export_save_pressed();
  void on_pb_source_dir_pressed();
  void on_le_source_dir_text_changed(QString text);

  // config
  void on_pb_save_config_pressed();
  void on_pb_load_config_pressed();

  // callbacks
  void scan_to_file_count_callback(const std_msgs::Int32::ConstPtr& count);

private:
  Ui::Mapping ui_;
  QMainWindow* widget_;
  std::vector<Parameter_widget*> pws;
  Scan_to_file_widget* stfw;

  void initWidgets();

  Mapping_manager* mapping_manager;
  Rosbag_reader* rosbag_reader;

  ros::NodeHandle n;
  ros::Subscriber count_sub;
};

}
#endif