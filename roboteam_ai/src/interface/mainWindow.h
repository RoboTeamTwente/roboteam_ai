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

namespace rtt {
namespace ai {
namespace interface {

class MainWindow : public QMainWindow {
    Q_OBJECT
    public:
        explicit MainWindow(QWidget* parent = nullptr);

    public slots:
        void updateWidgets();
    private:
        std::shared_ptr<Visualizer> visualizer;
        std::shared_ptr<QHBoxLayout> horizontalLayout;
        std::shared_ptr<QVBoxLayout> verticalLayout;
        std::shared_ptr<QTreeWidget> treeWidget;
        std::shared_ptr<QComboBox> select_robot;
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
        int amountOfRobots = 0;
        void addRootItem(bt::Node::Ptr parent, QTreeWidgetItem* QParent);

        std::map<QTreeWidgetItem*, bt::Node::Ptr> treeItemMapping;
        QColor getColorForStatus(bt::Node::Status status);
        int frame = 0;
};

} // interface
} // ai
} // rtt

#endif //ROBOTEAM_AI_MAINWINDOW_H
