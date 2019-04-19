//
// Created by mrlukasbos on 27-11-18.
//

#ifndef ROBOTEAM_AI_MAINWINDOW_H
#define ROBOTEAM_AI_MAINWINDOW_H

#include <QMainWindow>
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
#include <QtWidgets/QDoubleSpinBox>
#include "TreeVisualizerWidget.h"
#include "RobotsWidget.h"
#include "PidBox.h"

namespace rtt {
namespace ai {
namespace interface {

class MainWindow : public QMainWindow {
    Q_OBJECT
        FRIEND_TEST(MainWindowTest, it_displays_main_window);
        FRIEND_TEST(MainWindowTest, it_shows_the_visualizer_properly);
        FRIEND_TEST(TreeVisualizerTest, it_properly_displays_trees);
        FRIEND_TEST(TreeVisualizerTest, it_sets_proper_color_for_status);
    public:
        explicit MainWindow(QWidget* parent = nullptr);
        QString getSelectStrategyText() const;
        void setSelectStrategyText(QString text);

    public slots:
        void toggleOurColorParam();
        void toggleOurSideParam();

        void sendHaltSignal();
        void updatePause();
        void setUseReferee(bool useRef);
        void updateRobotsWidget();
        void updateTreeWidget();
        void updateKeeperTreeWidget();
        void setTimeOutTop(bool top);
        void setUsesKeeper(bool usekeeper);

        void setShowDebugValueInTerminal(bool showDebug);
        void refreshSignal();

    private:
        Visualizer* visualizer;
        QHBoxLayout* horizontalLayout;
        RobotsWidget* robotsLayout;
        QVBoxLayout* mainLayout;
        QVBoxLayout* vLayout;
        RobotsWidget* robotsWidget;
        TreeVisualizerWidget* treeWidget;
        TreeVisualizerWidget* keeperTreeWidget;

        QPushButton* haltBtn;
        QPushButton* refreshBtn;

        QPushButton* toggleColorBtn;
        QPushButton* toggleSideBtn;
        QComboBox* select_strategy;
        QComboBox* select_keeper_strategy;
        QComboBox* select_goalie;

private:
        PidBox* numTreePidBox;
        PidBox* forcePidBox;
        PidBox* basicPidBox;

        void configureCheckBox(QString title, QLayout* layout, const QObject* receiver, const char* method,
                bool defaultState = false);
        int amountOfSelectedRobots = 0;
        int robotsInField = 0;
        void setToggleColorBtnLayout() const;
        void setToggleSideBtnLayout() const;
};

} // interface
} // ai
} // rtt

#endif //ROBOTEAM_AI_MAINWINDOW_H
