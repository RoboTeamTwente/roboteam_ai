//
// Created by mrlukasbos on 27-11-18.
//

#include "interface/widgets/mainWindow.h"
#include "utilities/Constants.h"
#include <treeinterp/BTFactory.h>
#include "interface/api/Output.h"
#include "interface/widgets/RobotsWidget.h"
#include "interface/widgets/PidsWidget.h"
#include "interface/widgets/MainControlsWidget.h"
#include "interface/widgets/VisualizationSettingsWidget.h"
#include <QSplitter>
#include <interface/widgets/SettingsWidget.h>
#include <QtWidgets/QMenuBar>

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

    auto menu = new QMenuBar(this);
    this->setMenuBar(menu);
    auto fileMenu = menu->addMenu(tr("&File"));
    auto viewMenu = menu->addMenu(tr("&View"));
    viewMenu->addAction(tr("Visualization"));

    // the main controls widget for the most crucial buttons
    // changing strategies, goalie id, etc.
    auto mainControlsWidget = new MainControlsWidget(this);
    vLayout->addWidget(mainControlsWidget);


    auto behaviourTreeWidget = new QWidget(this);
    auto behaviourTreeWidgetLayout = new QVBoxLayout();
    // create widgets hidden under tabs
    treeWidget = new TreeVisualizerWidget(this);



    behaviourTreeWidgetLayout->addWidget(treeWidget);

    auto refreshHButtonsLayout = new QHBoxLayout();
    auto refreshBtn = new QPushButton("Soft refresh");
    QObject::connect(refreshBtn, SIGNAL(clicked()), this, SLOT(refreshSignal()));
    refreshHButtonsLayout->addWidget(refreshBtn);

    auto refreshJsonBtn = new QPushButton("Hard refresh");
    QObject::connect(refreshJsonBtn, SIGNAL(clicked()), this, SLOT(refreshJSONSignal()));
    refreshHButtonsLayout->addWidget(refreshJsonBtn);
    behaviourTreeWidgetLayout->addLayout(refreshHButtonsLayout);
    behaviourTreeWidget->setLayout(behaviourTreeWidgetLayout);



    keeperTreeWidget = new TreeVisualizerWidget(this);
    auto visualizationSettingsWidget = new VisualizationSettingsWidget(visualizer, this);
    auto settingsWidget = new SettingsWidget(this);

    auto pidWidget = new PidsWidget();
    robotsWidget = new RobotsWidget(this);
    refWidget = new RuleSetWidget(this);
    checkboxWidget = new CheckboxWidget(visualizer, this);

    // add the tab widget
    auto tabWidget = new QTabWidget;

    auto DataTabWidget = new QTabWidget;
    DataTabWidget->addTab(behaviourTreeWidget, tr("Behaviour trees"));
    DataTabWidget->addTab(keeperTreeWidget, tr("Keeper"));
    DataTabWidget->addTab(robotsWidget, tr("Robots"));
    DataTabWidget->addTab(refWidget, tr("GameStateManager"));
    tabWidget->addTab(DataTabWidget, tr("Data"));

    auto SettingsTabWidget = new QTabWidget;
    SettingsTabWidget->addTab(settingsWidget, tr("General settings"));
    SettingsTabWidget->addTab(visualizationSettingsWidget, tr("Visualisation Settings"));
    SettingsTabWidget->addTab(pidWidget, tr("PID"));
    SettingsTabWidget->addTab(checkboxWidget, tr("Other Settings"));
    tabWidget->addTab(SettingsTabWidget, tr("Settings"));

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
    timer->start(40); // 25fps

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

    robotsTimer->start(1000); // 1fps
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


void MainWindow::refreshSignal() {
    robotDealer::RobotDealer::refresh();
}

void MainWindow::refreshJSONSignal() {
    BTFactory::makeTrees();
    robotDealer::RobotDealer::refresh();
}


} // interface
} // ai
} // rtt

// QT performance improvement
#include "include/roboteam_ai/interface/widgets/moc_mainWindow.cpp"