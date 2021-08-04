//
// Created by mrlukasbos on 27-11-18.
//

#ifndef ROBOTEAM_AI_MAINWINDOW_H
#define ROBOTEAM_AI_MAINWINDOW_H

#include <QComboBox>
#include <QGroupBox>
#include <QLabel>
#include <QMainWindow>
#include <QRadioButton>
#include <QTreeWidget>
#include <QtGui>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDoubleSpinBox>
#include "world/World.hpp"
#include <iostream>
#include <memory>

#include "ManualControlWidget.h"
#include "QColor"
#include "QHBoxLayout"
#include "QPushButton"
#include "QTreeWidgetItemIterator"

namespace rtt {
class ApplicationManager;
}

namespace rtt::ai::interface {

class MainWindow : public QMainWindow {
    Q_OBJECT
    FRIEND_TEST(MainWindowTest, it_displays_main_window);
    FRIEND_TEST(MainWindowTest, it_shows_the_visualizer_properly);
    FRIEND_TEST(TreeVisualizerTest, it_properly_displays_trees);
    FRIEND_TEST(TreeVisualizerTest, it_sets_proper_color_for_status);

   public:
    explicit MainWindow(QWidget *parent = nullptr, rtt::ApplicationManager *manager = nullptr);

     static void clearLayout(QLayout *layout);

   private:
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *mainLayout;
    QVBoxLayout *vLayout;
    ManualControlWidget *manualControlWidget;

//    InvariantsWidget *invariantsWidget;
};

}  // namespace rtt::ai::interface

#endif  // ROBOTEAM_AI_MAINWINDOW_H
