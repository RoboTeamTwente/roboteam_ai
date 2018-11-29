//
// Created by mrlukasbos on 27-11-18.
//

#ifndef ROBOTEAM_AI_MAINWINDOW_H
#define ROBOTEAM_AI_MAINWINDOW_H

#include <QMainWindow>
#include "../utilities/Constants.h"
#include <iostream>
#include <memory>
#include <QtWidgets/QCheckBox>
#include "widget.h"
#include "QHBoxLayout"
#include "QPushButton"

namespace rtt {
namespace ai {
namespace interface {

class MainWindow : public QMainWindow {
Q_OBJECT
public:
    explicit MainWindow(QWidget * parent = nullptr);
    void updateWidget();
private:
    std::shared_ptr<Widget> visualizer;
    std::shared_ptr<QHBoxLayout> horizontalLayout;
    std::shared_ptr<QVBoxLayout> verticalLayout;
    std::shared_ptr<QPushButton> button1;
    std::shared_ptr<QPushButton> button2;
    std::shared_ptr<QCheckBox> cb_ids;
    std::shared_ptr<QCheckBox> cb_rolenames;
    std::shared_ptr<QCheckBox> cb_tacticnames;
    std::shared_ptr<QSpacerItem> vSpacer;
};

} // interface
} // ai
} // rtt

#endif //ROBOTEAM_AI_MAINWINDOW_H
