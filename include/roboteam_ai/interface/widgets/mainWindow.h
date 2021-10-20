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

#include "GraphWidget.h"
#include "ManualControlWidget.h"
#include "PidBox.h"
#include "QColor"
#include "QHBoxLayout"
#include "QPushButton"
#include "QTreeWidgetItemIterator"
#include "RobotsWidget.h"
#include "RuleSetWidget.h"
#include "STPVisualizerWidget.h"
#include "widget.h"
#include "PlaysWidget.hpp"

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

    // this function is useful everywhere
    static void configureCheckBox(const QString &title, QLayout *layout, const QObject *receiver, const char *method, bool defaultState = false);

    static void configureCheckableMenuItem(QString title, const QString &hint, QMenu *menu, const QObject *receiver, const char *method, bool defaultState);
    static void clearLayout(QLayout *layout);
    void updatePlay(stp::Play *play);

   signals:
    void updateStpWidgets();

   public slots:
    void updateRobotsWidget();
    void setPlayForRobot(std::string const &str, uint8_t id);
    void setTacticForRobot(std::string const &str, uint8_t id);
    void setKeeperRole(stp::Role *role, stp::Status state);

   private:
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *mainLayout;
    QVBoxLayout *vLayout;
    RobotsWidget *robotsWidget;
    RuleSetWidget *refWidget;
    ManualControlWidget *manualControlWidget;
    STPVisualizerWidget *stpWidget;
    STPVisualizerWidget *keeperStpWidget;
    Visualizer *visualizer;
    GraphWidget *graphWidget;
    PlaysWidget* playsWidget;
//    InvariantsWidget *invariantsWidget;
};

}  // namespace rtt::ai::interface

#endif  // ROBOTEAM_AI_MAINWINDOW_H
