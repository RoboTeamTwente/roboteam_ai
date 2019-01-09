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
#include "QColor"
#include "../bt/Node.hpp"
#include "QTreeWidgetItemIterator"
#include <QtGui>
#include <QRadioButton>
#include <QGroupBox>
#include <QLabel>

namespace rtt {
namespace ai {
namespace interface {

class MainWindow : public QMainWindow {
    Q_OBJECT
    public:
        explicit MainWindow(QWidget* parent = nullptr);

public slots:
    void updateWidgets();
    void updateRobotsWidget();
    void toggleOurColorParam();
    void toggleOurSideParam();
private:
    std::shared_ptr<Visualizer> visualizer;
    std::shared_ptr<QHBoxLayout> horizontalLayout;
    std::shared_ptr<QHBoxLayout> robotsLayout;
    std::shared_ptr<QVBoxLayout> mainLayout;
    std::shared_ptr<QVBoxLayout> verticalLayout;
    std::shared_ptr<QTreeWidget> treeWidget;
    std::shared_ptr<QPushButton> toggleColorBtn;
    std::shared_ptr<QPushButton> toggleSideBtn;
    std::shared_ptr<QComboBox> select_strategy;
    std::shared_ptr<QCheckBox> cb_referee;
    std::shared_ptr<QCheckBox> cb_rolenames;
    std::shared_ptr<QCheckBox> cb_tacticnames;
    std::shared_ptr<QCheckBox> cb_tacticcolors;
    std::shared_ptr<QCheckBox> cb_angles;
    std::shared_ptr<QCheckBox> cb_path;
    std::shared_ptr<QCheckBox> cb_path_all;
    std::shared_ptr<QCheckBox> cb_velocities;

    void configureCheckBox(std::shared_ptr<QCheckBox> checkbox, std::shared_ptr<QLayout> layout,
                const QObject* receiver, const char* method, bool defaultState = false);

    bool hasCorrectTree = false;
    void addRootItem(bt::Node::Ptr parent, QTreeWidgetItem * QParent);
    std::map<QTreeWidgetItem *, bt::Node::Ptr> treeItemMapping;
    QVBoxLayout * createRobotGroupItem(roboteam_msgs::WorldRobot robot);
    QColor getColorForStatus(bt::Node::Status status);
    void clearLayout(QLayout *layout);
    int amountOfSelectedRobots = 0;

};

} // interface
} // ai
} // rtt

#endif //ROBOTEAM_AI_MAINWINDOW_H
