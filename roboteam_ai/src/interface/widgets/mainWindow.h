//
// Created by mrlukasbos on 27-11-18.
//

#ifndef ROBOTEAM_AI_MAINWINDOW_H
#define ROBOTEAM_AI_MAINWINDOW_H


#include <QtWidgets/QMainWindow>
#include "gtest/gtest_prod.h"
class QHBoxLayout;
class QVBoxLayout;

namespace rtt {
namespace ai {
namespace interface {

class RobotsWidget;
class RuleSetWidget;
class TreeVisualizerWidget;
class Visualizer;
class MainWindow : public QMainWindow {
Q_OBJECT
    FRIEND_TEST(MainWindowTest, it_displays_main_window);
    FRIEND_TEST(MainWindowTest, it_shows_the_visualizer_properly);
    FRIEND_TEST(TreeVisualizerTest, it_properly_displays_trees);
    FRIEND_TEST(TreeVisualizerTest, it_sets_proper_color_for_status);
public:
    explicit MainWindow(QWidget* parent = nullptr);

    // this function is useful everywhere
    static void configureCheckBox(QString title, QLayout* layout, const QObject* receiver, const char* method,
                              bool defaultState = false);
    static void clearLayout(QLayout* layout);

public slots:
    void updateRobotsWidget();
    void updateTreeWidget();
    void updateKeeperTreeWidget();

private:
    QHBoxLayout* horizontalLayout;
    QVBoxLayout* mainLayout;
    QVBoxLayout* vLayout;
    RobotsWidget* robotsWidget;
    RuleSetWidget * refWidget;
    TreeVisualizerWidget* treeWidget;
    TreeVisualizerWidget* keeperTreeWidget;
    Visualizer* visualizer;
};

} // interface
} // ai
} // rtt

#endif //ROBOTEAM_AI_MAINWINDOW_H
