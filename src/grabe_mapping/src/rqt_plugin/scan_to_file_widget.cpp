#include "rqt_plugin/scan_to_file_widget.h"
#include <QFileDialog>

namespace grabe_mapping {

Scan_to_file_widget::Scan_to_file_widget(Rosbag_reader* rosbag_reader, QString title, QWidget *parent)
    : QWidget(parent), rosbag_reader(rosbag_reader) {
    
    this->ui.setupUi(this);

    this->ui.gb_scan_to_file->setTitle(title);
    this->ui.pb_cancel->setEnabled(false);

    // Connect slots
        // rosbag
    QObject::connect(ui.pb_fileDialog, &QPushButton::pressed, this, &Scan_to_file_widget::pb_fileDialog_pressed);
    QObject::connect(ui.rb_lefthanded, &QRadioButton::toggled, this, &Scan_to_file_widget::rb_lefthanded_toggled);
    QObject::connect(ui.rb_meter, &QRadioButton::toggled, this, &Scan_to_file_widget::rb_meter_toggled);
    QObject::connect(ui.le_filePath, &QLineEdit::textChanged, this, &Scan_to_file_widget::le_filePath_text_changed);

        // topics
    QObject::connect(ui.le_scan, &QLineEdit::textChanged, this, &Scan_to_file_widget::le_scan_text_changed);
    QObject::connect(ui.le_odom, &QLineEdit::textChanged, this, &Scan_to_file_widget::le_odom_text_changed);
    QObject::connect(ui.le_gps, &QLineEdit::textChanged, this, &Scan_to_file_widget::le_gps_text_changed);

        // output
    QObject::connect(ui.pb_output, &QPushButton::pressed, this, &Scan_to_file_widget::pb_output_pressed);
    QObject::connect(ui.le_output, &QLineEdit::textChanged, this, &Scan_to_file_widget::le_output_text_changed);

        // start/cancel
    QObject::connect(ui.pb_start, &QPushButton::pressed, this, &Scan_to_file_widget::pb_start_pressed);
    QObject::connect(ui.pb_cancel, &QPushButton::pressed, this, &Scan_to_file_widget::pb_cancel_pressed);

        // rosbag reader
    QObject::connect(this->rosbag_reader, &Rosbag_reader::finished, this, &Scan_to_file_widget::rosbag_reader_finished);
}

void Scan_to_file_widget::save_settings(qt_gui_cpp::Settings& instance_settings) const {
    // rosbag
    instance_settings.setValue("rosbag_filename", this->ui.le_filePath->text());
    instance_settings.setValue("input_is_meter", this->ui.rb_meter->isChecked());
    instance_settings.setValue("input_is_lefthanded", this->ui.rb_lefthanded->isChecked());
    // topics
    instance_settings.setValue("scan_topic", this->ui.le_scan->text());
    instance_settings.setValue("odom_topic", this->ui.le_odom->text());
    instance_settings.setValue("gps_topic", this->ui.le_gps->text());
    // output
    instance_settings.setValue("output_filepath", this->ui.le_output->text());
}

void Scan_to_file_widget::restore_settings(const qt_gui_cpp::Settings& instance_settings) {
    // rosbag
    if(instance_settings.contains("rosbag_filename"))
        this->ui.le_filePath->setText(instance_settings.value("rosbag_filename").toString());
    if(instance_settings.contains("input_is_meter"))
        this->ui.rb_meter->setChecked(instance_settings.value("input_is_meter").toBool());
    if(instance_settings.contains("input_is_lefthanded"))
        this->ui.rb_lefthanded->setChecked(instance_settings.value("input_is_lefthanded").toBool());
    // topics
    if(instance_settings.contains("scan_topic"))
        this->ui.le_scan->setText(instance_settings.value("scan_topic").toString());
    if(instance_settings.contains("odom_topic"))
        this->ui.le_odom->setText(instance_settings.value("odom_topic").toString());
    if(instance_settings.contains("gps_topic"))
        this->ui.le_gps->setText(instance_settings.value("gps_topic").toString());
    // output
    if(instance_settings.contains("output_filepath"))
        this->ui.le_output->setText(instance_settings.value("output_filepath").toString());
}

// slots

void Scan_to_file_widget::pb_fileDialog_pressed() {
    QString rosbag_filename = QFileDialog::getOpenFileName(
        this,
        tr("Select Bagfile"),
        "/home/jannik/Bachelorarbeit/Bagfiles",
        tr("*.bag")
    );

    this->ui.le_filePath->setText(rosbag_filename);
}

void Scan_to_file_widget::le_filePath_text_changed(QString text) {
    this->rosbag_reader->set_rosbag_filename(text);
}

void Scan_to_file_widget::rb_lefthanded_toggled() {
    this->rosbag_reader->toggle_input_is_lefthanded();
}

void Scan_to_file_widget::rb_meter_toggled() {
    this->rosbag_reader->toggle_input_is_meter();
}

// topics
void Scan_to_file_widget::le_scan_text_changed(QString text) {
    this->rosbag_reader->set_scan_topic(text);
}

void Scan_to_file_widget::le_odom_text_changed(QString text) {
    this->rosbag_reader->set_odom_topic(text);
}

void Scan_to_file_widget::le_gps_text_changed(QString text) {
    this->rosbag_reader->set_gps_topic(text);
}

void Scan_to_file_widget::le_scan_type_text_changed(QString text) {

}

void Scan_to_file_widget::le_odom_type_text_changed(QString text) {

}

void Scan_to_file_widget::le_gps_type_text_changed(QString text) {

}

// output
void Scan_to_file_widget::pb_output_pressed() {
    QString output_filepath = QFileDialog::getExistingDirectory(
        this,
        tr("Select Directory for output files"),
        "/home/jannik/Bachelorarbeit",
        QFileDialog::ShowDirsOnly
    );

    this->ui.le_output->setText(output_filepath);
}

void Scan_to_file_widget::le_output_text_changed(QString text) {
    if(text.at(text.size() - 1) != '/')
        text += '/';
    this->rosbag_reader->set_output_path(text);
}

// start / cancel
void Scan_to_file_widget::pb_start_pressed() {
    this->rosbag_reader->read();
    this->ui.pb_cancel->setEnabled(true);
    this->ui.pb_start->setEnabled(false);
}

void Scan_to_file_widget::pb_cancel_pressed() {
    this->rosbag_reader->cancel();
}

//rosbag reader
void Scan_to_file_widget::rosbag_reader_finished() {
    this->ui.pb_cancel->setEnabled(false);
    this->ui.pb_start->setEnabled(true);
}

}
