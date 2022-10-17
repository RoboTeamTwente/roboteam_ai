#include "interface/widgets/mainWindow.h"

#include <interface/widgets/SettingsWidget.h>

#include "interface/widgets/MainControlsWidget.h"
#include "interface/widgets/PidsWidget.h"
#include "interface/widgets/VisualizationSettingsWidget.h"

namespace rtt::ai::interface {

MainWindow::MainWindow(QWidget *parent, STPManager *manager) : QMainWindow(parent) {
    setMinimumWidth(800);
    setMinimumHeight(600);

    // layouts
    visualizer = new Visualizer(this);
    mainLayout = new QVBoxLayout();
    horizontalLayout = new QHBoxLayout();
    vLayout = new QVBoxLayout();

    auto menu = new QMenuBar(this);
    this->setMenuBar(menu);
    menu->addMenu(tr("&File"));
    auto viewMenu = menu->addMenu(tr("&Visualization"));

    // the main controls widget for the most crucial buttons
    // changing strategies, goalie id, etc.
    auto mainControlsWidget = new MainControlsWidget(this, manager);
    vLayout->addWidget(mainControlsWidget);

    auto behaviourTreeWidget = new QWidget(this);
    auto behaviourTreeWidgetLayout = new QVBoxLayout();
    // create widgets hidden under tabs
    stpWidget = new STPVisualizerWidget(this);

    behaviourTreeWidgetLayout->addWidget(stpWidget);
    behaviourTreeWidget->setLayout(behaviourTreeWidgetLayout);

    keeperStpWidget = new STPVisualizerWidget(this);
    auto visualizationSettingsWidget = new VisualizationSettingsWidget(visualizer, this);
    auto settingsWidget = new SettingsWidget(this);

    auto pidWidget = new PidsWidget();
    robotsWidget = new RobotsWidget(this);
    refWidget = new RuleSetWidget(this);
    manualControlWidget = new ManualControlWidget(this);

    // add the tab widget
    auto tabWidget = new QTabWidget;

    graphWidget = new GraphWidget(this);

    playsWidget = new PlaysWidget(this);

    auto DataTabWidget = new QTabWidget;
    DataTabWidget->addTab(behaviourTreeWidget, tr("STP states"));
    DataTabWidget->addTab(keeperStpWidget, tr("Keeper"));
    DataTabWidget->addTab(playsWidget, "Plays");
    DataTabWidget->addTab(graphWidget, tr("Charts"));
    DataTabWidget->addTab(robotsWidget, tr("Robots"));
    DataTabWidget->addTab(refWidget, tr("GameStateManager"));
    tabWidget->addTab(DataTabWidget, tr("Data"));

    auto SettingsTabWidget = new QTabWidget;
    SettingsTabWidget->addTab(settingsWidget, tr("General settings"));
    SettingsTabWidget->addTab(visualizationSettingsWidget, tr("Visualisation Settings"));
    SettingsTabWidget->addTab(pidWidget, tr("PID"));
    tabWidget->addTab(SettingsTabWidget, tr("Settings"));
    tabWidget->addTab(manualControlWidget, tr("Manual"));

    vLayout->addWidget(tabWidget);

    // set up the general layout structure
    // visualizer on the left, sidebar (with maincontrols and tabs) on the right.
    auto splitter = new QSplitter();  // the splitter is an horizontal view that allows to be changed by the user
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
    auto *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(20);  // 50fps

    // start the UI update cycles
    // these are slower than the tick rate
    auto *robotsTimer = new QTimer(this);
    connect(robotsTimer, SIGNAL(timeout()), refWidget, SLOT(updateContents()));
    connect(robotsTimer, SIGNAL(timeout()), this,
            SLOT(updateRobotsWidget()));  // we need to pass the visualizer so thats why a seperate function is used
    connect(robotsTimer, SIGNAL(timeout()), mainControlsWidget, SLOT(updateContents()));
    robotsTimer->start(500);  // 2fps

    auto *graphTimer = new QTimer(this);
    connect(graphTimer, SIGNAL(timeout()), graphWidget, SLOT(updateContents()));
    graphTimer->start(500);  // 2fps

    connect(this, &MainWindow::updateStpWidgets, stpWidget, &STPVisualizerWidget::outputStpData);
    connect(this, &MainWindow::updateStpWidgets, keeperStpWidget, &STPVisualizerWidget::outputStpData);
    connect(this, &MainWindow::updateStpWidgets, playsWidget, &PlaysWidget::updatePlays);
}

/// Set up a checkbox and add it to the layout
void MainWindow::configureCheckBox(const QString &title, QLayout *layout, const QObject *receiver, const char *method, bool defaultState) {
    auto checkbox = new QCheckBox(title);
    checkbox->setChecked(defaultState);
    layout->addWidget(checkbox);
    QObject::connect(checkbox, SIGNAL(clicked(bool)), receiver, method);
}

/// delete a layout and its children
void MainWindow::clearLayout(QLayout *layout) {
    QLayoutItem *item;
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
    std::optional<rtt::world::view::WorldDataView> currentWorld;
    {
        auto const &[_, world] = rtt::world::World::instance();
        if (!world) {
            std::cerr << "World is nullptr" << std::endl;
            return;
        }
        currentWorld = world->getWorld();
    }
    if (currentWorld) {
        robotsWidget->updateContents(visualizer, *currentWorld);
    }
}

void MainWindow::updatePlay(stp::Play *play) {
    stpWidget->updateContents(play);
    updateStpWidgets();
}

void MainWindow::setPlayForRobot(std::string const &str, uint8_t id) { visualizer->setPlayForRobot(str, id); }

void MainWindow::setKeeperRole(stp::Role *keeperRole, stp::Status state) { keeperStpWidget->updateKeeperContents(keeperRole, state); }

void MainWindow::updateProcessedVisionPackets(const std::vector<proto::SSL_WrapperPacket> &packets) { visualizer->updateProcessedVisionPackets(packets); }

}  // namespace rtt::ai::interface
