//
// Created by mrlukasbos on 27-11-18.
//

#include "mainWindow.h"
#include "roboteam_ai/src/utilities/Constants.h"
#include <roboteam_ai/src/treeinterp/BTFactory.h>
#include "roboteam_ai/src/interface/api/Output.h"
#include "RobotsWidget.h"
#include "PidsWidget.h"
#include "MainControlsWidget.h"
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
    auto mainControlsWidget = new MainControlsWidget(this);

    vLayout->addWidget(mainControlsWidget);


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



    connect(mainControlsWidget, SIGNAL(treeHasChanged()), treeWidget, SLOT(invalidateTree()));
    connect(mainControlsWidget, SIGNAL(treeHasChanged()), keeperTreeWidget, SLOT(invalidateTree()));


    // start the UI update cycles
    // these are slower
    auto * robotsTimer = new QTimer(this);
     connect(robotsTimer, SIGNAL(timeout()), this, SLOT(updateTreeWidget()));
    connect(robotsTimer, SIGNAL(timeout()), this, SLOT(updateKeeperTreeWidget()));
    connect(robotsTimer, SIGNAL(timeout()), this, SLOT(updateRobotsWidget())); // we need to pass the visualizer so thats why a seperate function is used
    connect(robotsTimer, SIGNAL(timeout()), mainControlsWidget, SLOT(updatePause()));
    robotsTimer->start(200); // 5fps
}


/// Set up a checkbox and add it to the layout
void MainWindow::configureCheckBox(QString title, QLayout * layout, const QObject* receiver, const char* method,
        bool defaultState) {

    auto checkbox = new QCheckBox(title);
    checkbox->setChecked(defaultState);
    layout->addWidget(checkbox);
    QObject::connect(checkbox, SIGNAL(clicked(bool)), receiver, method);
}


void MainWindow::updateRobotsWidget() {
    if (world::world->weHaveRobots())
        robotsWidget->updateContents(visualizer);
}

void MainWindow::setShowDebugValueInTerminal(bool showDebug) {
    Output::setShowDebugValues(showDebug);
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



} // interface
} // ai
} // rtt
