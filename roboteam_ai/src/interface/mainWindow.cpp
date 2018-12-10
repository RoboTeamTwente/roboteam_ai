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

    // selectionbox for selecting a robot from a dropdown
    select_robot = std::make_shared<QComboBox>();
    verticalLayout->addWidget(select_robot.get());
    QObject::connect(select_robot.get(), SIGNAL(currentIndexChanged(int)), visualizer.get(), SLOT(selectRobot(int)));
    // TODO this might not work properly on robocup where different IDs can be used

    // functions to select strategies
    cb_referee = std::make_shared<QCheckBox>("Use referee");
    configureCheckBox(cb_referee, verticalLayout, visualizer.get(), SLOT(setShowRoles(bool)), constants::STD_SHOW_ROLES);

    select_strategy = std::make_shared<QComboBox>();
    verticalLayout->addWidget(select_strategy.get());
    for (std::string const &strategyName : Switches::strategyJsonFileNames) {
        select_strategy->addItem(QString::fromStdString(strategyName));
    }

    QObject::connect(select_strategy.get(), QOverload<const QString &>::of(&QComboBox::currentIndexChanged), [=](const QString &strategyName){
        // http://doc.qt.io/qt-5/qcombobox.html#currentIndexChanged-1
        BTFactory::getFactory().setCurrentTree(strategyName.toStdString());
        hasCorrectTree = false;
    });


    // Checkboxes for the visualization
    cb_rolenames = std::make_shared<QCheckBox>("show rolenames");
    configureCheckBox(cb_rolenames, verticalLayout, visualizer.get(), SLOT(setShowRoles(bool)), constants::STD_SHOW_ROLES);

    cb_tacticnames = std::make_shared<QCheckBox>("show tacticnames");
    configureCheckBox(cb_tacticnames, verticalLayout, visualizer.get(), SLOT(setShowTactics(bool)), constants::STD_SHOW_TACTICS);

    cb_tacticcolors = std::make_shared<QCheckBox>("show tacticColors");
    configureCheckBox(cb_tacticcolors, verticalLayout, visualizer.get(), SLOT(setShowTacticColors(bool)), constants::STD_SHOW_TACTICS_COLORS);

    cb_angles = std::make_shared<QCheckBox>("show angles");
    configureCheckBox(cb_angles, verticalLayout, visualizer.get(), SLOT(setShowAngles(bool)), constants::STD_SHOW_ANGLES);

    cb_velocities = std::make_shared<QCheckBox>("show velocities");
    configureCheckBox(cb_velocities, verticalLayout, visualizer.get(), SLOT(setShowVelocities(bool)), constants::STD_SHOW_VELOCITIES);

    cb_path = std::make_shared<QCheckBox>("show path for current robot");
    configureCheckBox(cb_path, verticalLayout, visualizer.get(), SLOT(setShowPath(bool)), constants::STD_SHOW_PATHS_CURRENT);

    cb_path_all = std::make_shared<QCheckBox>("show path for all robots");
    configureCheckBox(cb_path_all, verticalLayout, visualizer.get(), SLOT(setShowPathAll(bool)), constants::STD_SHOW_PATHS_ALL);


    // set up tree widget
    treeWidget = std::make_shared<QTreeWidget>();
    treeWidget->setColumnCount(2);
    treeWidget->setColumnWidth(0, 250);

    verticalLayout->addWidget(treeWidget.get());

    // main layout: left the visualizer and right the vertical layout
    horizontalLayout->addWidget(visualizer.get(), 3); // width stretch 3/5
    horizontalLayout->addLayout(verticalLayout.get(), 2); // width stretch 2/5

    // apply layout
    setCentralWidget(new QWidget);
    centralWidget()->setLayout(horizontalLayout.get());

    // start the UI update cycles
    auto *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    connect(timer, SIGNAL(timeout()), this, SLOT(updateWidgets()));
    timer->start(20); // 50fps
}

// some widgets need to be updated regularly
void MainWindow::updateWidgets() {
    // Iterate through all treeWidget items to update the status if needed
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

    // initiate a redraw when the actual tree and the tree in the widget are not the same
    std::string currentTree = BTFactory::getFactory().getCurrentTree();
    if (QString::fromStdString(currentTree) != select_strategy->currentText()) {
        hasCorrectTree = false;
        select_strategy->setCurrentText(QString::fromStdString(currentTree));
    }

    // if the tree did change, clear the treewidget and rebuild it
    if (!hasCorrectTree && BTFactory::getFactory().isInitialized()) {
        treeItemMapping.clear();
        treeWidget->clear();
        bt::BehaviorTree::Ptr tree = BTFactory::getFactory().getTree(BTFactory::getFactory().getCurrentTree());
        auto treeItemRoot = new QTreeWidgetItem(treeWidget.get());
        treeItemRoot->setText(0, QString::fromStdString(tree->GetRoot()->node_name()));
        treeItemRoot->setText(1, QString::fromStdString(statusToString(tree->GetRoot()->getStatus())));
        treeItemRoot->setBackgroundColor(1, getColorForStatus(tree->GetRoot()->getStatus()));

        addRootItem(tree->GetRoot(), treeItemRoot);
        treeWidget->expandAll();
        treeWidget->update();
        hasCorrectTree = true;
    }

    // if the amount of robots in the world size is not consistent with our knowledge, refresh the select box
    if (amountOfRobots != World::get_world().us.size()) {
        select_robot->clear();
        for (auto robot : World::get_world().us) {
            select_robot->addItem(QString::number(robot.id));
        }
        amountOfRobots = static_cast<int>(World::get_world().us.size());
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

/// Set up the checkbox properties for a given checkbox
void MainWindow::configureCheckBox(std::shared_ptr<QCheckBox> checkbox, std::shared_ptr<QLayout> layout,
                                   const QObject *receiver, const char * method,
                                   bool defaultState) {
    checkbox->setChecked(defaultState);
    layout->addWidget(checkbox.get());
    QObject::connect(checkbox.get(), SIGNAL(clicked(bool)), receiver, method);
}

} // interface
} // ai
} // rtt