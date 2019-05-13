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
#include "VisualizationSettingsWidget.h"
#include <QSplitter>

namespace rtt {
namespace ai {
namespace interface {

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setMinimumWidth(800);
    setMinimumHeight(600);

    // layouts
    visualizer = new Visualizer(this);
    mainLayout = new QVBoxLayout();
    horizontalLayout = new QHBoxLayout();
    vLayout = new QVBoxLayout();

    // the main controls widget for the most crucial buttons
    // changing strategies, goalie id, etc.
    auto mainControlsWidget = new MainControlsWidget(this);
    vLayout->addWidget(mainControlsWidget);

    // create widgets hidden under tabs
    treeWidget = new TreeVisualizerWidget(this);
    keeperTreeWidget = new TreeVisualizerWidget(this);
    auto visualizationSettingsWidget = new VisualizationSettingsWidget(visualizer, this);
    auto pidWidget = new PidsWidget();
    robotsWidget = new RobotsWidget(this);

    // add the tab widget
    auto tabWidget = new QTabWidget;
    tabWidget->addTab(treeWidget, tr("Behaviour trees"));
    tabWidget->addTab(keeperTreeWidget, tr("Keeper"));
    tabWidget->addTab(visualizationSettingsWidget, tr("Visualisation Settings"));
    tabWidget->addTab(pidWidget, tr("PID"));
    tabWidget->addTab(robotsWidget, tr("Robots"));
    vLayout->addWidget(tabWidget);

    // set up the general layout structure
    // visualizer on the left, sidebar (with maincontrols and tabs) on the right.
    auto splitter = new QSplitter(); // the splitter is an horizontal view that allows to be changed by the user
    splitter->addWidget(visualizer);
    auto sideBarWidget = new QWidget;
    sideBarWidget->setLayout(vLayout);
    splitter->addWidget(sideBarWidget);
    splitter->setSizes({600, 200});
    horizontalLayout->addWidget(splitter);
    mainLayout->addLayout(horizontalLayout);

    // apply layout to the window
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
    // these are slower than the tick rate
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

// when updating the robotswidget it needs the current visualizer state
void MainWindow::updateRobotsWidget() {
    if (world::world->weHaveRobots())
        robotsWidget->updateContents(visualizer);
}

// update the tree widget with the newest strategy tree
void MainWindow::updateTreeWidget() {
    this->treeWidget->updateContents(BTFactory::getTree(BTFactory::getCurrentTree()));
}

// update the keeper widget with the newest keeper tree
void MainWindow::updateKeeperTreeWidget() {
   this->keeperTreeWidget->updateContents(BTFactory::getKeeperTree());
}


} // interface
} // ai
} // rtt
