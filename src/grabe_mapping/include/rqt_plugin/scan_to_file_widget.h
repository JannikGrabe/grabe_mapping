#ifndef GRABE_MAPPING_SCAN_TO_FILE_WIDGET_H
#define GRABE_MAPPING_SCAN_TO_FILE_WIDGET_H

#include <QtGui>
#include <QWidget>
#include <grabe_mapping/ui_scan_to_file_widget.h>

namespace grabe_mapping {

class Scan_to_file_widget : public QWidget {
    Q_OBJECT

private:
    Ui::Scan_to_file_widget ui;

public:

    Scan_to_file_widget(QString title, QWidget *parent = nullptr) : QWidget(parent) {
        this->ui.setupUi(this);
    }
};

}

#endif
