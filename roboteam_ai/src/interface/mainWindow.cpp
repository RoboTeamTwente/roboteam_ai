//
// Created by mrlukasbos on 27-11-18.
//

#include "mainWindow.h"
#include "../utilities/Constants.h"
#include <roboteam_ai/src/treeinterp/BTFactory.h>
#include "InterfaceValues.h"

namespace rtt {
namespace ai {
namespace interface {

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setMinimumWidth(800);
    setMinimumHeight(600);

    visualizer = new Visualizer(this);
    mainLayout = new QVBoxLayout();
    horizontalLayout = new QHBoxLayout();
    verticalLayout = new QVBoxLayout();
    robotsLayout = new QHBoxLayout();

    // functions to select strategies
    configureCheckBox("Use referee", verticalLayout, visualizer, SLOT(setShowRoles(bool)),
            constants::STD_SHOW_ROLES);

    select_strategy = new QComboBox();
    verticalLayout->addWidget(select_strategy);
    for (std::string const &strategyName : Switches::strategyJsonFileNames) {
        select_strategy->addItem(QString::fromStdString(strategyName));
    }

    auto haltBtn = new QPushButton("HALT");
    QObject::connect(haltBtn, SIGNAL(clicked()), this, SLOT(sendHaltSignal()));
    verticalLayout->addWidget(haltBtn);

    toggleColorBtn = new QPushButton("Color");
    QObject::connect(toggleColorBtn, SIGNAL(clicked()), this, SLOT(toggleOurColorParam()));
    verticalLayout->addWidget(toggleColorBtn);
    
    toggleSideBtn = new QPushButton("Side");
    QObject::connect(toggleSideBtn, SIGNAL(clicked()), this, SLOT(toggleOurSideParam()));


    verticalLayout->addWidget(toggleSideBtn);

    doubleSpinBoxesGroup = new QGroupBox("GoToPosLuth PID options");
    spinBoxLayout =new QHBoxLayout();

    sb_luth_P = new QDoubleSpinBox();
    sb_luth_P->setRange(-20, 20);
    sb_luth_P->setSingleStep(0.1f);
    sb_luth_P->setValue(InterfaceValues::getLuthP());
    QObject::connect(sb_luth_P, SIGNAL(valueChanged(double)), this, SLOT(updatePID_luth()));

    spinBoxLayout->addWidget(sb_luth_P);

    sb_luth_I = new QDoubleSpinBox();
    sb_luth_I->setRange(-20, 20);
    sb_luth_I->setSingleStep(0.1f);
    sb_luth_I->setValue(InterfaceValues::getLuthI());
    QObject::connect(sb_luth_I, SIGNAL(valueChanged(double)), this, SLOT(updatePID_luth()));

    spinBoxLayout->addWidget(sb_luth_I);

    sb_luth_D = new QDoubleSpinBox();
    sb_luth_D->setRange(-20, 20);
    sb_luth_D->setSingleStep(0.1f);
    sb_luth_D->setValue(InterfaceValues::getLuthD());
    QObject::connect(sb_luth_D, SIGNAL(valueChanged(double)), this, SLOT(updatePID_luth()));

    spinBoxLayout->addWidget(sb_luth_D);

    doubleSpinBoxesGroup->setLayout(spinBoxLayout);
    verticalLayout->addWidget(doubleSpinBoxesGroup);

    QObject::connect(select_strategy, static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::currentIndexChanged),
            [=](const QString &strategyName) {
              // http://doc.qt.io/qt-5/qcombobox.html#currentIndexChanged-1
              BTFactory::setCurrentTree(strategyName.toStdString());
              hasCorrectTree = false;
            });

    configureCheckBox("show rolenames", verticalLayout, visualizer, SLOT(setShowRoles(bool)),
            constants::STD_SHOW_ROLES);

    configureCheckBox("show tacticnames", verticalLayout, visualizer, SLOT(setShowTactics(bool)),
            constants::STD_SHOW_TACTICS);

    configureCheckBox("show tacticColors", verticalLayout, visualizer, SLOT(setShowTacticColors(bool)),
            constants::STD_SHOW_TACTICS_COLORS);

    configureCheckBox("show angles", verticalLayout, visualizer, SLOT(setShowAngles(bool)),
            constants::STD_SHOW_ANGLES);

    configureCheckBox("show velocities", verticalLayout, visualizer, SLOT(setShowVelocities(bool)),
            constants::STD_SHOW_VELOCITIES);

    configureCheckBox("show path for current robot", verticalLayout, visualizer, SLOT(setShowPath(bool)),
            constants::STD_SHOW_PATHS_CURRENT);

    configureCheckBox("show path for all robots", verticalLayout, visualizer, SLOT(setShowPathAll(bool)),
            constants::STD_SHOW_PATHS_ALL);

    configureCheckBox("Show marker for Ball Placement", verticalLayout, visualizer, SLOT(setShowBallPlacementMarker(bool)),
            constants::STD_SHOW_BALL_PLACEMENT_MARKER);

    // set up tree widget
    treeWidget = new QTreeWidget();
    treeWidget->setColumnCount(2);
    treeWidget->setColumnWidth(0, 250);

    verticalLayout->addWidget(treeWidget);

    // main layout: left the visualizer and right the vertical layout
    horizontalLayout->addWidget(visualizer, 3); // width stretch 3/5
    horizontalLayout->addLayout(verticalLayout, 2); // width stretch 2/5

    mainLayout->addLayout(horizontalLayout, 5); // height stretch 5/6
    mainLayout->addLayout(robotsLayout, 1); // height stretch 1/6


    // apply layout
    setCentralWidget(new QWidget);
    centralWidget()->setLayout(mainLayout);

    // start the UI update cycles
    auto* timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    connect(timer, SIGNAL(timeout()), this, SLOT(updateWidgets()));
    timer->start(20); // 50fps

    // start the UI update cycles
    auto * robotsTimer = new QTimer(this);
    connect(robotsTimer, SIGNAL(timeout()), this, SLOT(updateRobotsWidget()));
    robotsTimer->start(200); // 5fps
    
    delete timer;
    delete robotsTimer;
}

// some widgets need to be updated regularly
void MainWindow::updateWidgets() {
    // Iterate through all treeWidget items to update the status if needed
    QTreeWidgetItemIterator iter(treeWidget, QTreeWidgetItemIterator::All);
    while (*iter) {
        QTreeWidgetItem* widgetItem = *iter;
        if (treeItemMapping.find(widgetItem) != treeItemMapping.end()) {
            bt::Node::Ptr item = treeItemMapping.at(widgetItem);
            QString status = QString::fromStdString(statusToString(item->getStatus()));
            if (widgetItem->text(1) != status) {
                widgetItem->setText(1, status);
                widgetItem->setBackgroundColor(1, getColorForStatus(item->getStatus()));
            }
        }
        ++ iter;
        delete widgetItem;
    }

    // initiate a redraw when the actual tree and the tree in the widget are not the same
    std::string currentTree = BTFactory::getFactory().getCurrentTree();
    if (QString::fromStdString(currentTree) != select_strategy->currentText()) {
        hasCorrectTree = false;
        select_strategy->setCurrentText(QString::fromStdString(currentTree));
    }

    // if the tree did change, clear the treewidget and rebuild it
    if (! hasCorrectTree && BTFactory::getFactory().isInitialized()) {
        treeItemMapping.clear();
        treeWidget->clear();
        bt::BehaviorTree::Ptr tree = BTFactory::getFactory().getTree(BTFactory::getFactory().getCurrentTree());

        if (tree && tree->GetRoot()) {
            auto treeItemRoot = new QTreeWidgetItem(treeWidget);
            treeItemRoot->setText(0, QString::fromStdString(tree->GetRoot()->node_name()));
            treeItemRoot->setText(1, QString::fromStdString(statusToString(tree->GetRoot()->getStatus())));
            treeItemRoot->setBackgroundColor(1, getColorForStatus(tree->GetRoot()->getStatus()));

            addRootItem(tree->GetRoot(), treeItemRoot);
            treeWidget->expandAll();
            treeWidget->update();
        }
        hasCorrectTree = true;
    }
}

void MainWindow::updateRobotsWidget() {

    auto us = World::get_world().us;

    // reload the widgets completely if a robot is added or removed
    // or if the amount of selected robots is not accurate
    if (robotsLayout->count() != static_cast<int>(us.size()) || amountOfSelectedRobots != static_cast<int>(visualizer->getSelectedRobots().size())) {
        amountOfSelectedRobots = visualizer->getSelectedRobots().size();
        clearLayout(robotsLayout);

        for (auto robot : us) {
            QGroupBox * groupBox = new QGroupBox("Robot " + QString::number(robot.id));
            groupBox->setCheckable(true);
            groupBox->setChecked(visualizer->robotIsSelected(robot));
            QObject::connect(groupBox, &QGroupBox::clicked, [=]() {
                visualizer->toggleSelectedRobot(robot.id);
            });
            groupBox->setLayout(createRobotGroupItem(robot));
            robotsLayout->addWidget(groupBox, 1);
        }
    } else {
        for (int i = 0; i < static_cast<int>(us.size()); i++) {
            if (robotsLayout->itemAt(i)) {
                auto robotwidget = robotsLayout->itemAt(i)->widget();
                clearLayout(robotwidget->layout());
                delete robotwidget->layout();
                if (!robotwidget->layout()) {
                    robotwidget->setLayout(createRobotGroupItem(us.at(i)));
                }
            }
        }
    }
}


/// Use recursion to iterate through the children of each node
void MainWindow::addRootItem(bt::Node::Ptr parent, QTreeWidgetItem* QParent) {
    for (auto const &child : parent->getChildren()) {
        auto treeItemchild = new QTreeWidgetItem(QParent);
        treeItemchild->setText(0, QString::fromStdString(child->node_name()));
        treeItemchild->setText(1, QString::fromStdString(statusToString(child->getStatus())));

        std::pair<QTreeWidgetItem*, bt::Node::Ptr> pair{treeItemchild, child};
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
void MainWindow::configureCheckBox(QString title, QLayout * layout,
        const QObject* receiver, const char* method,
        bool defaultState) {

    QCheckBox * checkbox = new QCheckBox(title);
    checkbox->setChecked(defaultState);
    layout->addWidget(checkbox);
    QObject::connect(checkbox, SIGNAL(clicked(bool)), receiver, method);
}


QVBoxLayout * MainWindow::createRobotGroupItem(roboteam_msgs::WorldRobot robot) {
    auto vbox = new QVBoxLayout();

    auto velLabel = new QLabel("vel: (x = " + QString::number(robot.vel.x, 'G', 3) + ", y = " + QString::number(robot.vel.y, 'g', 3) + ") m/s");
    velLabel->setFixedWidth(250);
    vbox->addWidget(velLabel);

    auto angleLabel = new QLabel("angle: " + QString::number(robot.angle, 'g', 3) + " radians");
    angleLabel->setFixedWidth(250);
    vbox->addWidget(angleLabel);

    auto posLabel = new QLabel("pos: (x = " + QString::number(robot.pos.x, 'g', 3) + ", y = " + QString::number(robot.pos.y, 'g', 3) + ")");
    posLabel->setFixedWidth(250);
    vbox->addWidget(posLabel);

    auto wLabel = new QLabel("w: " + QString::number(robot.w, 'g', 3)  + "rad/s");
    wLabel->setFixedWidth(250);
    vbox->addWidget(wLabel);

    return vbox;
}

void MainWindow::clearLayout(QLayout *layout) {
    QLayoutItem * item;
    while((item = layout->takeAt(0))) {
        if (item->layout()) {
            clearLayout(item->layout());
            delete item->layout();
        }
        if (item->widget()) {
            delete item->widget();
        }
        delete item;
    }
}


void MainWindow::toggleOurColorParam() {
    ros::NodeHandle nh;
    std::string ourColorParam, newParam;
    nh.getParam("our_color", ourColorParam);
    newParam = ourColorParam == "yellow" ? "blue" : "yellow";
    nh.setParam("our_color", newParam);
    toggleColorBtn->setText(QString::fromStdString(newParam));
}

void MainWindow::toggleOurSideParam() {
    ros::NodeHandle nh;
    std::string ourSideParam, newParam;
    nh.getParam("our_side", ourSideParam);
    newParam = ourSideParam == "right" ? "left" : "right";
    nh.setParam("our_side", newParam);
    toggleSideBtn->setText(QString::fromStdString(newParam));
}

void MainWindow::updatePID_luth() {
    InterfaceValues::setLuthP(sb_luth_P->value());
    InterfaceValues::setLuthI(sb_luth_I->value());
    InterfaceValues::setLuthD(sb_luth_D->value());
}

void MainWindow::sendHaltSignal() {
    InterfaceValues::sendHaltCommand();
}

} // interface
} // ai
} // rtt
