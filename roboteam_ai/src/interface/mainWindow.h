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
    void updatePID_luth();
    void sendHaltSignal();
    void setUseReferee(bool useRef);
    void updateRobotsWidget();
private:
    Visualizer * visualizer;
    QHBoxLayout * horizontalLayout;
    RobotsWidget * robotsLayout;
    QVBoxLayout * mainLayout;
    QVBoxLayout * vLayout;
    RobotsWidget * robotsWidget;
    TreeVisualizerWidget * treeWidget;
    QPushButton * haltBtn;

    QPushButton * toggleColorBtn;
    QPushButton * toggleSideBtn;
    QComboBox * select_strategy;

private:
    QGroupBox * doubleSpinBoxesGroup;
    QHBoxLayout * spinBoxLayout;
    QDoubleSpinBox * sb_luth_P;
    QDoubleSpinBox * sb_luth_I;
    QDoubleSpinBox * sb_luth_D;

    void configureCheckBox(QString title, QLayout * layout, const QObject* receiver, const char* method, bool defaultState = false);
    int amountOfSelectedRobots = 0;
};

} // interface
} // ai
} // rtt

#endif //ROBOTEAM_AI_MAINWINDOW_H
