#ifndef GRABE_MAPPING_GUI_PLUGIN_H
#define GRABE_MAPPING_GUI_PLUGIN_H

#include <rqt_gui_cpp/plugin.h>
#include <grabe_mapping/ui_gui_plugin.h>
#include <QWidget>
#include <QMainWindow>
#include "mapping/mapping.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "std_msgs/Int32.h"

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
  // rosbag
  void on_cb_use_output_files_state_changed(int state);
  void on_pb_fileDialog_pressed();
  void on_rb_lefthanded_toggled();
  void on_rb_meter_toggled();
  void on_le_filePath_text_changed(QString text);

  // work
  void on_pb_start_pressed();
  void on_work_finished(int exit_code);
  void on_pb_show_pressed();

  // topics
  void on_le_scan_text_changed(QString text);
  void on_le_odom_text_changed(QString text);
  void on_le_gps_text_changed(QString text);
  void on_le_scan_type_text_changed(QString text);
  void on_le_odom_type_text_changed(QString text);
  void on_le_gps_type_text_changed(QString text);

  // parameters
      // general
  void on_sb_first_value_changed(int val);
  void on_sb_last_value_changed(int val);
  void on_dbs_min_value_changed(double val);
  void on_dbs_max_value_changed(double val);
  void on_cb_correspondances_current_text_changed(QString text);
  void on_cb_metascan_state_changed(int state);
  void on_cb_export_state_changed(int state);
  void on_pb_export_pressed();

  void on_cb_icp_minimization_current_text_changed(QString text);
  void on_cb_nn_current_text_changed(QString text);
  void on_cb_closing_loop_current_text_changed(QString text);
  void on_cb_graphslam_current_text_changed(QString text);

      // icp parameters
  void on_sb_icp_iterations_value_changed(int val);
  void on_dsb_icp_epsilon_value_changed(double val);

      // nearest neighbor parameters
  void on_dsb_nn_p2p_distance_value_changed(double val);

      // other icp params
  void on_cb_metascan_toggled();

  // output
  void on_pb_output_pressed();
  void on_le_output_text_changed(QString text);

  // callbacks
  void scan_to_file_count_callback(const std_msgs::Int32::ConstPtr& count);

private:
  Ui::Mapping ui_;
  QMainWindow* widget_;
  
  void initWidgets();
  void initComboBoxes();
  static int runCommand(std::string command);

  Mapping* mapping;

  ros::NodeHandle n;
  ros::Subscriber count_sub;
};
}
#endif