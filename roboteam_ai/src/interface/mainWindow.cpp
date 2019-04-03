//
// Created by mrlukasbos on 27-11-18.
//

#include "mainWindow.h"
#include "../utilities/Constants.h"
#include <roboteam_ai/src/treeinterp/BTFactory.h>
#include "InterfaceValues.h"
#include "RobotsWidget.h"
#include <QSplitter>

namespace rtt {
namespace ai {
namespace interface {

MainWindow::MainWindow(QWidget* parent)
        :QMainWindow(parent) {

    // initialize values for interface to display
    InterfaceValues::setNumTreePosP(Constants::standardNumTreePosP());
    InterfaceValues::setNumTreePosI(Constants::standardNumTreePosI());
    InterfaceValues::setNumTreePosD(Constants::standardNumTreePosD());

    InterfaceValues::setUseRefereeCommands(Constants::STD_USE_REFEREE());

    setMinimumWidth(800);
    setMinimumHeight(600);

    // layouts
    visualizer = new Visualizer(this);
    mainLayout = new QVBoxLayout();
    horizontalLayout = new QHBoxLayout();
    vLayout = new QVBoxLayout();

    // set up the large widgets
    auto splitter = new QSplitter(); // the splitter is an horizontal view that allows to be changed by the user
    robotsWidget = new RobotsWidget(this);
    treeWidget = new TreeVisualizerWidget(this);


    // functions to select strategies
    configureCheckBox("Use referee", vLayout, this, SLOT(setUseReferee(bool)), Constants::STD_USE_REFEREE());

    select_strategy = new QComboBox();
    vLayout->addWidget(select_strategy);
    for (std::string const &strategyName : Switches::strategyJsonFileNames) {
        select_strategy->addItem(QString::fromStdString(strategyName));
    }

    auto hButtonsLayout = new QHBoxLayout();

    haltBtn = new QPushButton("Pause");
    QObject::connect(haltBtn, SIGNAL(clicked()), this, SLOT(sendHaltSignal()));
    hButtonsLayout->addWidget(haltBtn);
    haltBtn->setStyleSheet("background-color: #cc0000;");

    refreshBtn = new QPushButton("Refresh");
    QObject::connect(refreshBtn, SIGNAL(clicked()), this, SLOT(refreshSignal()));
    hButtonsLayout->addWidget(refreshBtn);
    refreshBtn->setStyleSheet("background-color: #0000cc;");

    toggleColorBtn = new QPushButton("Color");
    QObject::connect(toggleColorBtn, SIGNAL(clicked()), this, SLOT(toggleOurColorParam()));
    hButtonsLayout->addWidget(toggleColorBtn);
    setToggleColorBtnLayout(); // set the btn color and text to the current our_color

    vLayout->addLayout(hButtonsLayout);
    doubleSpinBoxesGroup_Pos_PID = new QGroupBox("GoToPosLuth Position PID options");
    spinBoxLayout =new QHBoxLayout();

    sb_luth_Pos_P = new QDoubleSpinBox();
    sb_luth_Pos_P->setRange(-20, 20);
    sb_luth_Pos_P->setSingleStep(0.1f);
    sb_luth_Pos_P->setValue(InterfaceValues::getNumTreePosP());
    QObject::connect(sb_luth_Pos_P, SIGNAL(valueChanged(double)), this, SLOT(updatePID_luth()));
    spinBoxLayout->addWidget(sb_luth_Pos_P);

    sb_luth_Pos_I = new QDoubleSpinBox();
    sb_luth_Pos_I->setRange(-20, 20);
    sb_luth_Pos_I->setSingleStep(0.1f);
    sb_luth_Pos_I->setValue(InterfaceValues::getNumTreePosI());
    QObject::connect(sb_luth_Pos_I, SIGNAL(valueChanged(double)), this, SLOT(updatePID_luth()));
    spinBoxLayout->addWidget(sb_luth_Pos_I);

    sb_luth_Pos_D = new QDoubleSpinBox();
    sb_luth_Pos_D->setRange(-20, 20);
    sb_luth_Pos_D->setSingleStep(0.1f);
    sb_luth_Pos_D->setValue(InterfaceValues::getNumTreePosD());
    QObject::connect(sb_luth_Pos_D, SIGNAL(valueChanged(double)), this, SLOT(updatePID_luth()));
    spinBoxLayout->addWidget(sb_luth_Pos_D);
    doubleSpinBoxesGroup_Pos_PID->setLayout(spinBoxLayout);

    QObject::connect(select_strategy, static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::currentIndexChanged),
            [=](const QString &strategyName) {
              // http://doc.qt.io/qt-5/qcombobox.html#currentIndexChanged-1
              BTFactory::makeTrees();
              BTFactory::setCurrentTree(strategyName.toStdString());
              treeWidget->setHasCorrectTree(false);
            });

    auto pidWidget = new QWidget;
    auto pidVLayout = new QVBoxLayout();
    pidVLayout->addWidget(doubleSpinBoxesGroup_Pos_PID);

    auto pidSpacer = new QSpacerItem(100, 100, QSizePolicy::Expanding, QSizePolicy::Expanding);
    pidVLayout->addSpacerItem(pidSpacer);
    pidWidget->setLayout(pidVLayout);

    auto checkboxWidget = new QWidget;

    auto cbVLayout = new QVBoxLayout();
    configureCheckBox("show rolenames", cbVLayout, visualizer, SLOT(setShowRoles(bool)), Constants::STD_SHOW_ROLES());
    configureCheckBox("show tacticnames", cbVLayout, visualizer, SLOT(setShowTactics(bool)), Constants::STD_SHOW_TACTICS());
    configureCheckBox("show tacticColors", cbVLayout, visualizer, SLOT(setShowTacticColors(bool)), Constants::STD_SHOW_TACTICS_COLORS());
    configureCheckBox("show angles", cbVLayout, visualizer, SLOT(setShowAngles(bool)), Constants::STD_SHOW_ANGLES());
    configureCheckBox("show velocities", cbVLayout, visualizer, SLOT(setShowVelocities(bool)), Constants::STD_SHOW_VELOCITIES());
    configureCheckBox("show path for selected robots", cbVLayout, visualizer, SLOT(setShowPath(bool)), Constants::STD_SHOW_PATHS_CURRENT());
    configureCheckBox("show path for all robots", cbVLayout, visualizer, SLOT(setShowPathAll(bool)), Constants::STD_SHOW_PATHS_ALL());
    configureCheckBox("Show marker for Ball Placement", cbVLayout, visualizer, SLOT(setShowBallPlacementMarker(bool)), Constants::STD_SHOW_BALL_PLACEMENT_MARKER());
    configureCheckBox("show debug values in terminal", cbVLayout, visualizer, SLOT(setShowDebugValueInTerminal(bool)), Constants::STD_SHOW_DEBUG_VALUES());
    configureCheckBox("show passes for selected robots", cbVLayout, visualizer, SLOT(setShowAvailablePasses(bool)), Constants::STD_SHOW_AVAILABLE_PASSES());

    auto cbVSpacer = new QSpacerItem(100, 100, QSizePolicy::Expanding, QSizePolicy::Expanding);
    cbVLayout->addSpacerItem(cbVSpacer);
    checkboxWidget->setLayout(cbVLayout);

    // add the tab widget
    auto tabWidget = new QTabWidget;
    tabWidget->addTab(treeWidget, tr("Behaviour trees"));
    tabWidget->addTab(checkboxWidget, tr("Visualisation Settings"));
    tabWidget->addTab(pidWidget, tr("PID"));
    tabWidget->addTab(robotsWidget, tr("Robots"));
    vLayout->addWidget(tabWidget);


    splitter->addWidget(visualizer);
    auto sideBarWidget = new QWidget;
    sideBarWidget->setLayout(vLayout);
    splitter->addWidget(sideBarWidget);
    splitter->setSizes({600, 200});

    horizontalLayout->addWidget(splitter);
    mainLayout->addLayout(horizontalLayout);


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
    connect(robotsTimer, SIGNAL(timeout()), this, SLOT(updatePause()));
    robotsTimer->start(200); // 5fps
}
void MainWindow::setToggleColorBtnLayout() const {
    ros::NodeHandle nh;
    std::string ourColorParam;
    nh.getParam("our_color", ourColorParam);
    if (ourColorParam == "yellow") {
        toggleColorBtn->setStyleSheet("background-color: orange;"); // orange is more readable
    } else {
        toggleColorBtn->setStyleSheet("background-color: blue;");
    }
    toggleColorBtn->setText(QString::fromStdString(ourColorParam));
}

/// Set up a checkbox and add it to the layout
void MainWindow::configureCheckBox(QString title, QLayout * layout, const QObject* receiver, const char* method,
        bool defaultState) {

    auto checkbox = new QCheckBox(title);
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

    setToggleColorBtnLayout();
}

/// update the PID values for gotopos Luth
void MainWindow::updatePID_luth() {
    InterfaceValues::setNumTreePosP(sb_luth_Pos_P->value());
    InterfaceValues::setNumTreePosI(sb_luth_Pos_I->value());
    InterfaceValues::setNumTreePosD(sb_luth_Pos_D->value());
}

/// send a halt signal to stop all trees from executing
void MainWindow::sendHaltSignal() {
    InterfaceValues::sendHaltCommand();
}

void MainWindow::updatePause() {
    rtt::ai::Pause pause;
    if (pause.getPause()) {
        haltBtn->setText("Resume");
        haltBtn->setStyleSheet("background-color: #00b200;");
    }
    else {
        haltBtn->setText("Pause");
        haltBtn->setStyleSheet("background-color: #cc0000;");
    }

}

void MainWindow::updateRobotsWidget() {
    robotsWidget->updateContents(visualizer);
}

void MainWindow::setShowDebugValueInTerminal(bool showDebug) {
    InterfaceValues::setShowDebugValues(showDebug);
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
void MainWindow::refreshSignal() {
    BTFactory::makeTrees();
    treeWidget->setHasCorrectTree(false);
}


} // interface
} // ai
} // rtt
