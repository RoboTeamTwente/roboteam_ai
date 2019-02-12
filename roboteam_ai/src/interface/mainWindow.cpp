//
// Created by mrlukasbos on 27-11-18.
//

#include "mainWindow.h"
#include "../utilities/Constants.h"
#include <roboteam_ai/src/treeinterp/BTFactory.h>
#include "InterfaceValues.h"
#include "RobotsWidget.h"

namespace rtt {
namespace ai {
namespace interface {

MainWindow::MainWindow(QWidget* parent)
        :QMainWindow(parent) {

    // initialize values for interface to display
    InterfaceValues::setLuthP(Constants::standard_luth_P());
    InterfaceValues::setLuthI(Constants::standard_luth_I());
    InterfaceValues::setLuthD(Constants::standard_luth_D());
    InterfaceValues::setUseRefereeCommands(Constants::STD_USE_REFEREE());

    setMinimumWidth(800);
    setMinimumHeight(600);

    visualizer = new Visualizer(this);
    mainLayout = new QVBoxLayout();
    horizontalLayout = new QHBoxLayout();
    vLayout = new QVBoxLayout();

    robotsWidget = new RobotsWidget(this);

    // functions to select strategies
    configureCheckBox("Use referee", vLayout, this, SLOT(setUseReferee(bool)), Constants::STD_USE_REFEREE());

    select_strategy = new QComboBox();
    vLayout->addWidget(select_strategy);
    for (std::string const &strategyName : Switches::strategyJsonFileNames) {
        select_strategy->addItem(QString::fromStdString(strategyName));
    }

    auto haltBtn = new QPushButton("HALT");
    QObject::connect(haltBtn, SIGNAL(clicked()), this, SLOT(sendHaltSignal()));
    vLayout->addWidget(haltBtn);

    toggleColorBtn = new QPushButton("Color");
    QObject::connect(toggleColorBtn, SIGNAL(clicked()), this, SLOT(toggleOurColorParam()));
    vLayout->addWidget(toggleColorBtn);
    
    toggleSideBtn = new QPushButton("Side");
    QObject::connect(toggleSideBtn, SIGNAL(clicked()), this, SLOT(toggleOurSideParam()));
    vLayout->addWidget(toggleSideBtn);

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
    vLayout->addWidget(doubleSpinBoxesGroup);

    QObject::connect(select_strategy, static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::currentIndexChanged),
            [=](const QString &strategyName) {
              // http://doc.qt.io/qt-5/qcombobox.html#currentIndexChanged-1
              BTFactory::setCurrentTree(strategyName.toStdString());
              treeWidget->setHasCorrectTree(false);
            });

    configureCheckBox("show rolenames", vLayout, visualizer, SLOT(setShowRoles(bool)), Constants::STD_SHOW_ROLES());
    configureCheckBox("show tacticnames", vLayout, visualizer, SLOT(setShowTactics(bool)), Constants::STD_SHOW_TACTICS());
    configureCheckBox("show tacticColors", vLayout, visualizer, SLOT(setShowTacticColors(bool)), Constants::STD_SHOW_TACTICS_COLORS());
    configureCheckBox("show angles", vLayout, visualizer, SLOT(setShowAngles(bool)), Constants::STD_SHOW_ANGLES());
    configureCheckBox("show velocities", vLayout, visualizer, SLOT(setShowVelocities(bool)), Constants::STD_SHOW_VELOCITIES());
    configureCheckBox("show path for current robot", vLayout, visualizer, SLOT(setShowPath(bool)), Constants::STD_SHOW_PATHS_CURRENT());
    configureCheckBox("show path for all robots", vLayout, visualizer, SLOT(setShowPathAll(bool)), Constants::STD_SHOW_PATHS_ALL());
    configureCheckBox("Show marker for Ball Placement", vLayout, visualizer, SLOT(setShowBallPlacementMarker(bool)), Constants::STD_SHOW_BALL_PLACEMENT_MARKER());

    // set up tree widget
    treeWidget = new TreeVisualizerWidget(this);
    vLayout->addWidget(treeWidget);

    // main layout: left the visualizer and right the vertical layout
    horizontalLayout->addWidget(visualizer, 3); // width stretch 3/5
    horizontalLayout->addLayout(vLayout, 2); // width stretch 2/5
    mainLayout->addLayout(horizontalLayout, 5); // height stretch 5/6
    mainLayout->addWidget(robotsWidget, 1); //robotswidget height 1/6


    // apply layout
    setCentralWidget(new QWidget);
    centralWidget()->setLayout(mainLayout);

    // start the UI update cycles
    // update mainwindow and field visualization
    auto* timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(20); // 50fps

    // start the UI update cycles
    // these are slower
    auto * robotsTimer = new QTimer(this);
    connect(robotsTimer, SIGNAL(timeout()), treeWidget, SLOT(updateContents()));
    connect(robotsTimer, SIGNAL(timeout()), this, SLOT(updateRobotsWidget())); // we need to pass the visualizer so thats why a seperate function is used
    robotsTimer->start(200); // 5fps
}

/// Set up a checkbox and add it to the layout
void MainWindow::configureCheckBox(QString title, QLayout * layout, const QObject* receiver, const char* method,
        bool defaultState) {

    QCheckBox * checkbox = new QCheckBox(title);
    checkbox->setChecked(defaultState);
    layout->addWidget(checkbox);
    QObject::connect(checkbox, SIGNAL(clicked(bool)), receiver, method);
}

/// toggle the ROS param 'our_color'
void MainWindow::toggleOurColorParam() {
    ros::NodeHandle nh;
    std::string ourColorParam, newParam;
    nh.getParam("our_color", ourColorParam);
    newParam = ourColorParam == "yellow" ? "blue" : "yellow";
    nh.setParam("our_color", newParam);
    toggleColorBtn->setText(QString::fromStdString(newParam));
}

/// toggle the ROS param 'our_side'
void MainWindow::toggleOurSideParam() {
    ros::NodeHandle nh;
    std::string ourSideParam, newParam;
    nh.getParam("our_side", ourSideParam);
    newParam = ourSideParam == "right" ? "left" : "right";
    nh.setParam("our_side", newParam);
    toggleSideBtn->setText(QString::fromStdString(newParam));
}

/// update the PID values for gotopos Luth
void MainWindow::updatePID_luth() {
    InterfaceValues::setLuthP(sb_luth_P->value());
    InterfaceValues::setLuthI(sb_luth_I->value());
    InterfaceValues::setLuthD(sb_luth_D->value());
}

/// send a halt signal to stop all trees from executing
void MainWindow::sendHaltSignal() {
    InterfaceValues::sendHaltCommand();
}

void MainWindow::updateRobotsWidget() {
    robotsWidget->updateContents(visualizer);
}

void MainWindow::setUseReferee(bool useRef) {
    InterfaceValues::setUseRefereeCommands(useRef);
}

QString MainWindow::getSelectStrategyText() const {
    return select_strategy->currentText();
}

void MainWindow::setSelectStrategyText(QString text) {
    select_strategy->setCurrentText(text);
}

} // interface
} // ai
} // rtt
