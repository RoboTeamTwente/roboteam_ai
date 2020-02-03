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
#include "TreeVisualizerWidget.h"
#include "widget.h"

namespace rtt::ai::interface {

    class MainWindow : public QMainWindow {
        Q_OBJECT
        FRIEND_TEST(MainWindowTest, it_displays_main_window);
        FRIEND_TEST(MainWindowTest, it_shows_the_visualizer_properly);
        FRIEND_TEST(TreeVisualizerTest, it_properly_displays_trees);
        FRIEND_TEST(TreeVisualizerTest, it_sets_proper_color_for_status);

        public:
        explicit MainWindow(QWidget *parent = nullptr);

        // this function is useful everywhere
        static void configureCheckBox(QString title, QLayout *layout, const QObject *receiver, const char *method, bool defaultState = false);

        static void configureCheckableMenuItem(QString title, QString hint, QMenu *menu, const QObject *receiver, const char *method, bool defaultState);
        static void clearLayout(QLayout *layout);

        public slots:
        void updateRobotsWidget();
        void updateTreeWidget();
        void updateKeeperTreeWidget();

        void refreshSignal();
        void refreshJSONSignal();

        private:
        QHBoxLayout *horizontalLayout;
        QVBoxLayout *mainLayout;
        QVBoxLayout *vLayout;
        RobotsWidget *robotsWidget;
        RuleSetWidget *refWidget;
        ManualControlWidget *manualControlWidget;
        TreeVisualizerWidget *treeWidget;
        TreeVisualizerWidget *keeperTreeWidget;
        Visualizer *visualizer;
        GraphWidget *graphWidget;
    };

}  // namespace rtt::ai::interface

#endif  // ROBOTEAM_AI_MAINWINDOW_H
