//
// Created by mrlukasbos on 27-11-18.
//

#include <roboteam_ai/src/utilities/Constants.h>
#include <roboteam_ai/src/treeinterp/BTFactory.h>
#include "mainWindow.h"

namespace rtt {
namespace ai {
namespace interface {

    MainWindow::MainWindow(QWidget * parent) : QMainWindow(parent) {
    setMinimumWidth(800);
    setMinimumHeight(600);

    visualizer = std::make_shared<Visualizer>(this);
    horizontalLayout = std::make_shared<QHBoxLayout>();
    verticalLayout = std::make_shared<QVBoxLayout>();

    // button for X
    select_robot = std::make_shared<QComboBox>();
    verticalLayout->addWidget(select_robot.get());

    // checkbox for toggling Role text
    cb_rolenames = std::make_shared<QCheckBox>("show rolenames");
    cb_rolenames->setChecked(constants::STD_SHOW_ROLES);
    verticalLayout->addWidget(cb_rolenames.get());
    QObject::connect(cb_rolenames.get(), SIGNAL(clicked(bool)), visualizer.get(), SLOT(setShowRoles(bool)));

    // checkbox for toggling Tactics text
    cb_tacticnames = std::make_shared<QCheckBox>("show tacticnames");
    cb_tacticnames->setChecked(constants::STD_SHOW_TACTICS);
    verticalLayout->addWidget(cb_tacticnames.get());
    QObject::connect(cb_tacticnames.get(), SIGNAL(clicked(bool)), visualizer.get(), SLOT(setShowTactics(bool)));

    // checkbox for toggling Tactics colors
    cb_tacticcolors = std::make_shared<QCheckBox>("show tacticColors");
    cb_tacticcolors->setChecked(constants::STD_SHOW_TACTICS_COLORS);
    verticalLayout->addWidget(cb_tacticcolors.get());
    QObject::connect(cb_tacticcolors.get(), SIGNAL(clicked(bool)), visualizer.get(), SLOT(setShowTacticColors(bool)));

    // checkbox for toggling angle indicators
    cb_angles = std::make_shared<QCheckBox>("show angles");
    cb_angles->setChecked(constants::STD_SHOW_ANGLES);
    verticalLayout->addWidget(cb_angles.get());
    QObject::connect(cb_angles.get(), SIGNAL(clicked(bool)), visualizer.get(), SLOT(setShowAngles(bool)));

    // checkbox for toggling velocity indicators
    cb_velocities = std::make_shared<QCheckBox>("show velocities");
    cb_velocities->setChecked(constants::STD_SHOW_VELOCITIES);
    verticalLayout->addWidget(cb_velocities.get());
    QObject::connect(cb_velocities.get(), SIGNAL(clicked(bool)), visualizer.get(), SLOT(setShowVelocities(bool)));

    // checkbox for toggling velocity indicators
    cb_path = std::make_shared<QCheckBox>("show path for current robot");
    cb_path->setChecked(constants::STD_SHOW_PATHS_CURRENT);
    verticalLayout->addWidget(cb_path.get());
    QObject::connect(cb_path.get(), SIGNAL(clicked(bool)), visualizer.get(), SLOT(setShowPath(bool)));

    // checkbox for toggling velocity indicators
    cb_path_all = std::make_shared<QCheckBox>("show path for all robots");
    cb_path_all->setChecked(constants::STD_SHOW_PATHS_ALL);
    verticalLayout->addWidget(cb_path_all.get());
    QObject::connect(cb_path_all.get(), SIGNAL(clicked(bool)), visualizer.get(), SLOT(setShowPathAll(bool)));

    // Spacer to nicely align buttons at the top
    vSpacer = std::make_shared<QSpacerItem>(0,10, QSizePolicy::Expanding, QSizePolicy::Expanding);
    verticalLayout->addItem(vSpacer.get());

    treeWidget = std::make_shared<QTreeWidget>();
    treeWidget->setColumnCount(2);



    verticalLayout->addWidget(treeWidget.get());



    // main layout: left the visualizer and right the vertical layout
    horizontalLayout->addWidget(visualizer.get(), 2); // width stretch 2/3
    horizontalLayout->addLayout(verticalLayout.get(), 1); // width stretch 1/3

    // apply layout
    setCentralWidget(new QWidget);
    centralWidget()->setLayout(horizontalLayout.get());
}

void MainWindow::updateWidget() {
    visualizer->update();
   // select_robot->setText(QString::number(visualizer->getSelectedRobot().id));

   if (select_robot->count() != World::get_world().us.size()) {
       select_robot->clear();
       for (roboteam_msgs::WorldRobot robot : World::get_world().us) {
           select_robot->addItem(QString::number(robot.id));
       }
   }


    if (!didLoad) {
        bt::BehaviorTree::Ptr tree = BTFactory::getFactory().getTree("SimpleStrategy");

        auto treeItemRoot = new QTreeWidgetItem(treeWidget.get());
        treeItemRoot->setText(0, QString::fromStdString(tree->node_name()));
        treeItemRoot->setText(1, QString::fromStdString(statusToString(tree->getStatus())));

        addRootItem(tree, treeItemRoot);

        treeWidget->expandAll();
        treeWidget->update();
        didLoad = true;
    }
}

void MainWindow::addRootItem(bt::Node::Ptr parent, QTreeWidgetItem * QParent) {
    auto treeItemchild = new QTreeWidgetItem(QParent);
    treeItemchild->setText(0, QString::fromStdString(parent->node_name()));
    treeItemchild->setText(1, QString::fromStdString(statusToString(parent->getStatus())));

    for(auto const &child : parent->getChildren()) {


        addRootItem(child, treeItemchild);
    }
}

}
}
} // interface