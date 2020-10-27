
#include <QObject>
#include <QWidget>
#include <QtConcurrent/QtConcurrent>

class Mapping : public QWidget {
    Q_OBJECT

private:

    // rosbag
    QString rosbag_filename;
    bool input_is_meter;
    bool input_is_lefthanded;

    // topic 
    QString scan_topic;
    QString odom_topic;
    QString gps_topic;

    // ouput
    QString output_filepath;

    // work 
    std::string script_path;
    QFutureWatcher<void> watcher;

    // member methods
    void start_rosbag();

    void start_scan_to_file();

    static int run_command(std::string command);

    void init_states();

    bool check_states();

public slots:
    void on_work_finished();
    
public: 

    Mapping();

    void start_mapping();

    // getter
        // rosbag
    QString get_rosbag_filename();
    bool get_input_is_meter();
    bool get_input_is_lefthanded();

        // topic
    QString get_scan_topic();
    QString get_odom_topic();
    QString get_gps_topic();

        // output
    QString get_output_filepath();

    // setter
        // rosbag
    void set_rosbag_filename(QString filename);
    void set_input_is_meter(bool input_is_meter);
    void toggle_input_is_meter();
    void set_input_is_lefthanded(bool input_is_lefthanded);
    void toggle_input_is_lefthanded();

        // topic
    void set_scan_topic(QString topic);
    void set_odom_topic(QString topic);
    void set_gps_topic(QString topic);

        // output
    void set_output_filepath(QString filename);


signals:
    void finished_mapping(int exit_code);
};