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
#include <QComboBox>
#include "widget.h"
#include "QHBoxLayout"
#include "QPushButton"
#include <QTreeWidget>
#include "../bt/Node.hpp"

namespace rtt {
namespace ai {
namespace interface {

class MainWindow : public QMainWindow {
Q_OBJECT
public:
    explicit MainWindow(QWidget * parent = nullptr);
    void updateWidget();
private:
    std::shared_ptr<Visualizer> visualizer;
    std::shared_ptr<QHBoxLayout> horizontalLayout;
    std::shared_ptr<QVBoxLayout> verticalLayout;
    std::shared_ptr<QTreeWidget> treeWidget;
    std::shared_ptr<QComboBox> select_robot;
    std::shared_ptr<QCheckBox> cb_rolenames;
    std::shared_ptr<QCheckBox> cb_tacticnames;
    std::shared_ptr<QCheckBox> cb_tacticcolors;
    std::shared_ptr<QCheckBox> cb_angles;
    std::shared_ptr<QCheckBox> cb_path;
    std::shared_ptr<QCheckBox> cb_path_all;
    std::shared_ptr<QCheckBox> cb_velocities;
    std::shared_ptr<QSpacerItem> vSpacer;

    bool didLoad = false;
    void addRootItem(bt::Node::Ptr parent, QTreeWidgetItem * QParent);
};

} // interface
} // ai
} // rtt

#endif //ROBOTEAM_AI_MAINWINDOW_H
