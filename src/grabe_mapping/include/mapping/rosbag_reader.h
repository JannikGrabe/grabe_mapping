#include <QWidget>
#include <QtConcurrent/QtConcurrent>

namespace grabe_mapping {
    
class Rosbag_reader : public QWidget {
    Q_OBJECT

private:
    QString rosbag_filename;
    QString scan_topic;
    QString odom_topic;
    QString gps_topic;
    QString output_path;

    bool input_is_meter = false;
    bool input_is_lefthanded = false;
    bool cancelled = false;

    QFutureWatcher<void> watcher;

    static int run_command(std::string command);

public:
    Rosbag_reader();

    std::vector<std::string> check_states();

    void read();

    void cancel();

public slots:
    void on_finished();

signals:
    void started();
    void finished();
    void output_path_changed(QString path);

public: // setter
    void set_rosbag_filename(QString filename);
    void set_output_path(QString path);
    void set_scan_topic(QString topic);
    void set_odom_topic(QString topic);
    void set_gps_topic(QString topic);
    void toggle_input_is_meter();
    void toggle_input_is_lefthanded();
};

}