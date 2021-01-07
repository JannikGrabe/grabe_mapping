
#include <QObject>
#include <QWidget>
#include <QtConcurrent/QtConcurrent>
#include <map>
#include <QProcess>
#include "parameter_map.h"
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

class Mapping : public QWidget {
    Q_OBJECT

private:

    // rosbag
    bool use_rosbag;
    QString rosbag_filename;
    bool input_is_meter;
    bool input_is_lefthanded;
    int file_count;

    // topic 
    QString scan_topic;
    QString odom_topic;
    QString gps_topic;

    // ouput
    QString output_filepath;

    // Algorithms
    grabe_mapping::Parameter_map* parameters;

    // work 
    std::string script_path;
    QString export_path;
    QFutureWatcher<void> watcher;
    void (Mapping::*next_process)();
    bool cancelled;
    std::vector<std::string> icp_results;

    // Mapping
    void start_scan_to_file();

    void finish_scan_to_file();

    void start_slam6D();

    void finish_mapping();

    void read_results();

    static int run_command(std::string command);

    void stopChildProcesses(qint64 parentProcessId);

    // States
    void init_states();

    bool check_states();

    // Algorithms
    void initAlgorithms();

    
public: 

    Mapping();

    // Mapping
    void start_mapping();

    void cancel_mapping();

    void showResults();

    // PointCloud stuff
    void transform_cloud(pcl::PointCloud<pcl::PointXYZI> *in, pcl::PointCloud<pcl::PointXYZI> *out, double *angles, double *translation);

    void calculate_crispnesses(int scan1, int scan2);

    double calculate_crispness(pcl::PointCloud<pcl::PointXYZI> *in);

    void segmentPointCloud();
    
    // getter
        // rosbag
    bool get_use_rosbag() const;
    QString get_rosbag_filename() const;
    bool get_input_is_meter() const;
    bool get_input_is_lefthanded() const;
    int get_file_count() const;

        // topics
    QString get_scan_topic() const;
    QString get_odom_topic() const;
    QString get_gps_topic() const;

        // output
    QString get_output_filepath() const;

        // work
    std::vector<std::string> get_icp_results() const;

    // setter
        // rosbag
    void set_use_rosbag(bool state);
    void set_rosbag_filename(QString filename);
    void set_input_is_meter(bool input_is_meter);
    void toggle_input_is_meter();
    void set_input_is_lefthanded(bool input_is_lefthanded);
    void toggle_input_is_lefthanded();
    void set_file_count(int val);

        // topics
    void set_scan_topic(QString topic);
    void set_odom_topic(QString topic);
    void set_gps_topic(QString topic);

        // parameters
    bool set_parameter_value(std::string name, double value);
    bool set_parameter_value(std::string name, int value);
    bool set_parameter_value(std::string name, std::string value);
    bool set_parameter_active(std::string name, bool state);
    bool toggle_parameter_active(std::string name);
    void set_export_path(QString text);

        // output
    void set_output_filepath(QString filename);

signals:
    void finished_mapping(int exit_code);
    void finished_rosbag();

public slots:
    void on_process_finished();
};