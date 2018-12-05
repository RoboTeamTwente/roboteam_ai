//
// Created by mrlukasbos on 27-11-18.
//

#include <roboteam_ai/src/utilities/Constants.h>
#include <roboteam_ai/src/treeinterp/BTFactory.h>
#include <roboteam_ai/src/bt/Node.hpp>
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
    // vSpacer = std::make_shared<QSpacerItem>(0,10, QSizePolicy::Expanding, QSizePolicy::Expanding);
    // verticalLayout->addItem(vSpacer.get());

    treeWidget = std::make_shared<QTreeWidget>();
    treeWidget->setColumnCount(2);

    verticalLayout->addWidget(treeWidget.get());

    // main layout: left the visualizer and right the vertical layout
    horizontalLayout->addWidget(visualizer.get(), 3); // width stretch 3/5
    horizontalLayout->addLayout(verticalLayout.get(), 2); // width stretch 2/5

    // apply layout
    setCentralWidget(new QWidget);
    centralWidget()->setLayout(horizontalLayout.get());

    bt::BehaviorTree::Ptr tree = BTFactory::getFactory().getTree("GetBallTestStrategy"); // TODO notify tree change and reload the widget
    auto treeItemRoot = new QTreeWidgetItem(treeWidget.get());
    treeItemRoot->setText(0, QString::fromStdString(tree->GetRoot()->node_name()));
    treeItemRoot->setText(1, QString::fromStdString(statusToString(tree->GetRoot()->getStatus())));
    treeItemRoot->setBackgroundColor(1, getColorForStatus(tree->GetRoot()->getStatus()));

    addRootItem(tree->GetRoot(), treeItemRoot);
    treeWidget->expandAll();
    treeWidget->update();

    // start the UI update cycle
    auto *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    connect(timer, SIGNAL(timeout()), this, SLOT(updateWidget()));
    timer->start(20); // 50fps
}

void MainWindow::updateWidget() {
    QTreeWidgetItemIterator iter(treeWidget.get(), QTreeWidgetItemIterator::All);

    while (*iter) {
        QTreeWidgetItem *widgetItem = *iter;
        if (treeItemMapping.find(widgetItem) != treeItemMapping.end()) {
            bt::Node::Ptr item = treeItemMapping.at(widgetItem);
            QString status = QString::fromStdString(statusToString(item->getStatus()));
            if (widgetItem->text(1) != status) {
                widgetItem->setText(1, status);
                widgetItem->setBackgroundColor(1, getColorForStatus(item->getStatus()));
            }
        }
        ++iter;
    }
}

/// Use recursion to iterate through the children of each node
void MainWindow::addRootItem(bt::Node::Ptr parent, QTreeWidgetItem * QParent) {
    for(auto const &child : parent->getChildren()) {
        auto treeItemchild = new QTreeWidgetItem(QParent);
        treeItemchild->setText(0, QString::fromStdString(child->node_name()));
        treeItemchild->setText(1, QString::fromStdString(statusToString(child->getStatus())));

        std::pair<QTreeWidgetItem *, bt::Node::Ptr> pair{treeItemchild, child};
        treeItemMapping.insert(pair);

        treeItemchild->setBackgroundColor(1, getColorForStatus(child->getStatus()));
        QParent->addChild(treeItemchild);
        addRootItem(child, treeItemchild);
    }
}

/// returns a color for a given node status
QColor MainWindow::getColorForStatus(bt::Node::Status status) {
    switch (status) {
        case bt::Node::Status::Failure:
            return Qt::red;
        case bt::Node::Status::Running:
            return {"#99ff99"}; // light green
        case bt::Node::Status::Success:
            return {"#339933"}; // dark green
        case bt::Node::Status::Waiting:
            return Qt::gray;
        default:
            return Qt::white;
    }
}

}
}
} // interface