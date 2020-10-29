
#include <QObject>
#include <QWidget>
#include <QtConcurrent/QtConcurrent>
#include "mapping_algorithm.h"
#include <map>

class Mapping : public QWidget {
    Q_OBJECT

private:

    // rosbag
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

    // ICP
    MappingAlgorithm minimization;
    MappingAlgorithm nearest_neighbor;

    // Algorithms
    std::map<std::string, MappingAlgorithm> minimization_algorithms;
    std::map<std::string, MappingAlgorithm> nearest_neighbor_algorithms;

    // work 
    std::string script_path;
    QFutureWatcher<void> watcher;
    void (Mapping::*next_process)();

    // Mapping
    void start_scan_to_file();

    void start_slam6D();

    void showResults();

    void finish_mapping();

    static int run_command(std::string command);

    // States
    void init_states();

    bool check_states();

    // Algorithms
    void initAlgorithms();

    
public: 

    Mapping();

    // Mapping
    void start_mapping();

    // getter
        // rosbag
    QString get_rosbag_filename();
    bool get_input_is_meter();
    bool get_input_is_lefthanded();
    int get_file_count();

        // topics
    QString get_scan_topic();
    QString get_odom_topic();
    QString get_gps_topic();

        // ICP
    MappingAlgorithm get_minimization();
    MappingAlgorithm get_nearest_neighbor();

        // output
    QString get_output_filepath();

    // setter
        // rosbag
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

        // ICP
    bool set_minimization(QString text);
    bool set_nearest_neighbor(QString text);

        // output
    void set_output_filepath(QString filename);


signals:
    void finished_mapping(int exit_code);

public slots:
    void on_process_finished();
};