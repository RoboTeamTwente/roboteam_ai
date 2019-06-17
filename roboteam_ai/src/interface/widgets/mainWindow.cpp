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
    refWidget = new RuleSetWidget(this);
    checkboxWidget = new CheckboxWidget(visualizer, this);

    // add the tab widget
    auto tabWidget = new QTabWidget;

    auto DataTabWidget = new QTabWidget;
    DataTabWidget->addTab(treeWidget, tr("Behaviour trees"));
    DataTabWidget->addTab(keeperTreeWidget, tr("Keeper"));
    DataTabWidget->addTab(robotsWidget, tr("Robots"));
    DataTabWidget->addTab(refWidget, tr("GameStateManager"));
    tabWidget->addTab(DataTabWidget, tr("data"));

    auto SettingsTabWidget = new QTabWidget;
    SettingsTabWidget->addTab(visualizationSettingsWidget, tr("Visualisation Settings"));
    SettingsTabWidget->addTab(pidWidget, tr("PID"));
    SettingsTabWidget->addTab(checkboxWidget, tr("Other Settings"));
    tabWidget->addTab(SettingsTabWidget, tr("settings"));

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
    connect(robotsTimer, SIGNAL(timeout()), refWidget, SLOT(updateContents()));
    connect(robotsTimer, SIGNAL(timeout()), this, SLOT(updateRobotsWidget())); // we need to pass the visualizer so thats why a seperate function is used
    connect(robotsTimer, SIGNAL(timeout()), mainControlsWidget, SLOT(updatePause()));
    connect(robotsTimer, SIGNAL(timeout()), mainControlsWidget, SLOT(updateContents()));

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

/// delete a layout and its children
void MainWindow::clearLayout(QLayout* layout)
{
    QLayoutItem* item;
    while ((item = layout->takeAt(0))) {
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

// QT performance improvement
#include "moc_mainWindow.cpp"