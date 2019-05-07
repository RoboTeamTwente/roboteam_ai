//
// Created by mrlukasbos on 27-11-18.
//

#include "mainWindow.h"
#include "roboteam_ai/src/utilities/Constants.h"
#include <roboteam_ai/src/treeinterp/BTFactory.h>
#include "roboteam_ai/src/interface/api/Output.h"
#include "RobotsWidget.h"
#include "PidsWidget.h"
#include <QSplitter>

namespace rtt {
namespace ai {
namespace interface {

    enum showType {
        ALWAYS,
        SELECTED_ROBOTS,
        NEVER
    };

    enum VisualizationType {
        DEBUG,
        PATHFINDING,
        INTERCEPTING,
        SHOOTING,
        POSITIONING
    };

MainWindow::MainWindow(QWidget* parent)
        :QMainWindow(parent) {

    // initialize values for interface to display
    Output::setNumTreePid(Constants::standardNumTreePID());
    Output::setForcePid(Constants::standardForcePID());
    Output::setBasicPid(Constants::standardBasicPID());

    Output::setUseRefereeCommands(Constants::STD_USE_REFEREE());

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
    keeperTreeWidget = new TreeVisualizerWidget(this);


    // functions to select strategies
    configureCheckBox("Use referee", vLayout, this, SLOT(setUseReferee(bool)), Constants::STD_USE_REFEREE());

    select_strategy = new QComboBox();
    vLayout->addWidget(select_strategy);
    for (std::string const &strategyName : Switches::strategyJsonFileNames) {
        select_strategy->addItem(QString::fromStdString(strategyName));
    }

    select_keeper_strategy = new QComboBox();
    vLayout->addWidget(select_keeper_strategy);
    for (std::string const &keeperTacticName : Switches::keeperJsonFiles) {
        select_keeper_strategy->addItem(QString::fromStdString(keeperTacticName));
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

    toggleSideBtn = new QPushButton("Side");
    QObject::connect(toggleSideBtn, SIGNAL(clicked()), this, SLOT(toggleOurSideParam()));
    hButtonsLayout->addWidget(toggleSideBtn);
    setToggleColorBtnLayout(); // set the btn color and text to the current our_side

    vLayout->addLayout(hButtonsLayout);

    configureCheckBox("TimeOut to top", vLayout, this, SLOT(setTimeOutTop(bool)), Constants::STD_TIMEOUT_TO_TOP());
    configureCheckBox("Use keeper (does not work when referee used)", vLayout, this, SLOT(setUsesKeeper(bool)), robotDealer::RobotDealer::usesSeparateKeeper());
    
    QObject::connect(select_strategy, static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::activated),
            [=](const QString &strategyName) {
              // http://doc.qt.io/qt-5/qcombobox.html#currentIndexChanged-1
              BTFactory::setCurrentTree(strategyName.toStdString());
              robotDealer::RobotDealer::refresh();

              // the pointers of the trees have changed so the widgets should be notified about this
              treeWidget->setHasCorrectTree(false);
              keeperTreeWidget->setHasCorrectTree(false);
            });

    QObject::connect(select_keeper_strategy, static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::activated),
                     [=](const QString &keeperStrategyName) {
                         // http://doc.qt.io/qt-5/qcombobox.html#currentIndexChanged-1
                         BTFactory::setKeeperTree(keeperStrategyName.toStdString());
                         robotDealer::RobotDealer::refresh();

                         // the pointers of the trees have changed so the widgets should be notified about this
                         treeWidget->setHasCorrectTree(false);
                         keeperTreeWidget->setHasCorrectTree(false);
                     });



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
    configureCheckBox("Inverse interface", cbVLayout, visualizer, SLOT(setToggleFieldDirection(bool)), false);

    auto cbVSpacer = new QSpacerItem(100, 100, QSizePolicy::Expanding, QSizePolicy::Expanding);
    cbVLayout->addSpacerItem(cbVSpacer);
    checkboxWidget->setLayout(cbVLayout);

    // create a keeper widget
    auto keeperWidget = new QWidget;
    auto keeperVLayout = new QVBoxLayout(keeperWidget);

    select_goalie = new QComboBox();
    keeperVLayout->addWidget(select_goalie);
    QObject::connect(select_goalie, static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::currentIndexChanged),
                     [=](const QString &goalieId) {
                         // http://doc.qt.io/qt-5/qcombobox.html#currentIndexChanged-1
                         robotDealer::RobotDealer::setKeeperID(goalieId.toInt());
                         keeperTreeWidget->setHasCorrectTree(false);

                     });

    keeperVLayout->addWidget(select_goalie);
    keeperVLayout->addWidget(keeperTreeWidget);

    auto keeperVSpacer = new QSpacerItem(100, 100, QSizePolicy::Expanding, QSizePolicy::Expanding);
    keeperVLayout->addSpacerItem(keeperVSpacer);

    // add the tab widget
    auto tabWidget = new QTabWidget;
    tabWidget->addTab(treeWidget, tr("Behaviour trees"));
    tabWidget->addTab(checkboxWidget, tr("Visualisation Settings"));

    auto pidWidget = new PidsWidget();
    tabWidget->addTab(pidWidget, tr("PID"));
    tabWidget->addTab(robotsWidget, tr("Robots"));
    tabWidget->addTab(keeperWidget, tr("Keeper"));

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
     connect(robotsTimer, SIGNAL(timeout()), this, SLOT(updateTreeWidget()));
    connect(robotsTimer, SIGNAL(timeout()), this, SLOT(updateKeeperTreeWidget()));
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

void MainWindow::setToggleSideBtnLayout() const {
    ros::NodeHandle nh;
    std::string ourSideParam;
    nh.getParam("our_side", ourSideParam);
    if (ourSideParam == "left") {
        toggleSideBtn->setStyleSheet("background-color: #cc0000;");
        toggleSideBtn->setText("◀ Left");

    } else {
        toggleSideBtn->setText("right ▶");
        toggleSideBtn->setStyleSheet("background-color: #cc0000;");
    }
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

/// toggle the ROS param 'our_color'
    void MainWindow::toggleOurSideParam() {
        ros::NodeHandle nh;
        std::string ourColorParam, newParam;
        nh.getParam("our_side", ourColorParam);
        newParam = ourColorParam == "left" ? "right" : "left";
        nh.setParam("our_side", newParam);

        setToggleSideBtnLayout();
 }

/// send a halt signal to stop all trees from executing
void MainWindow::sendHaltSignal() {
    Output::sendHaltCommand();
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
    if (world::world->weHaveRobots())
        robotsWidget->updateContents(visualizer);
}

void MainWindow::setShowDebugValueInTerminal(bool showDebug) {
    Output::setShowDebugValues(showDebug);
}

void MainWindow::setUseReferee(bool useRef) {
    Output::setUseRefereeCommands(useRef);
}

QString MainWindow::getSelectStrategyText() const {
    return select_strategy->currentText();
}

void MainWindow::setSelectStrategyText(QString text) {
    select_strategy->setCurrentText(text);
}
void MainWindow::refreshSignal() {
    robotDealer::RobotDealer::refresh();
    keeperTreeWidget->setHasCorrectTree(false);
    treeWidget->setHasCorrectTree(false);
}

void MainWindow::updateTreeWidget() {
    this->treeWidget->updateContents(BTFactory::getTree(BTFactory::getCurrentTree()));
}

void MainWindow::updateKeeperTreeWidget() {

    if (robotsInField != static_cast<int>(world::world->getUs().size())) {
        select_goalie->clear();
        robotsInField = world::world->getUs().size();

        for (auto robot : world::world->getUs()) {
            std::string txt = to_string(robot.id);
            select_goalie->addItem(QString::fromStdString(txt));
        }
    }

   this->keeperTreeWidget->updateContents(BTFactory::getKeeperTree());
}

void MainWindow::setTimeOutTop(bool top) {
    rtt::ai::interface::Output::setTimeOutTop(top);
}

void MainWindow::setUsesKeeper(bool usekeeper) {
    robotDealer::RobotDealer::setUseSeparateKeeper(usekeeper);
    robotDealer::RobotDealer::refresh();
}


} // interface
} // ai
} // rtt
