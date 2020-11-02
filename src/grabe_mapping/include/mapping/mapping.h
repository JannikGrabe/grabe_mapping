
#include <QObject>
#include <QWidget>
#include <QtConcurrent/QtConcurrent>
#include "mapping_algorithm.h"
#include <map>
#include <QProcess>

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

    // ICP
    MappingAlgorithm icp_minimization;
    MappingAlgorithm nearest_neighbor;
    std::map<std::string, MappingAlgorithm> icp_minimization_algorithms;
    std::map<std::string, MappingAlgorithm> nearest_neighbor_algorithms;

        // ICP parameters
    int icp_max_iterations;
    double icp_epsilon;

        // nearest neighbor parameters
    double nn_max_p2p_distance;

        // other parameters
    bool match_meta_scan;

    // GraphSlam
    MappingAlgorithm closing_loop;
    MappingAlgorithm graphslam;
    std::map<std::string, MappingAlgorithm> closing_loop_algorithms;
    std::map<std::string, MappingAlgorithm> graphslam_algorithms;

        // closing loop parameters
    int loop_size;
    int max_distance;
    int min_overlap;
    int cl_max_p2p_distance;
    int cl_max_iterations;

        // graphSlam minimization parameters
    int graph_max_iterations;
    double graph_epsilon;
    double graph_max_p2p_distance;

    // work 
    std::string script_path;
    QFutureWatcher<void> watcher;
    void (Mapping::*next_process)();

    // Mapping
    void start_scan_to_file();

    void start_slam6D();

    void finish_mapping();

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

        // Algorithms
    MappingAlgorithm get_icp_minimization() const;
    MappingAlgorithm get_nearest_neighbor() const;
    MappingAlgorithm get_closing_loop() const;
    MappingAlgorithm get_graphslam() const;

        // output
    QString get_output_filepath() const;

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

        // Algorithms
    bool set_icp_minimization(QString text);
    bool set_nearest_neighbor(QString text);
    bool set_closing_loop(QString text);
    bool set_graphslam(QString text);

        // parameters
            // icp
    void set_icp_max_iterations(int it);
    void set_icp_epsilon(double eps);
            // nearest neighbor
    void set_nn_max_p2p_distance(double dist);
            // other icp parameters
    void set_match_meta_scan(bool state);
    void toggle_match_meta_scan();

        // output
    void set_output_filepath(QString filename);


signals:
    void finished_mapping(int exit_code);

public slots:
    void on_process_finished();
};