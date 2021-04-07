#include "interface/widgets/mainWindow.h"

#include <QMenuBar>
#include <QSplitter>
namespace rtt::ai::interface {

MainWindow::MainWindow(QWidget *parent, ApplicationManager *manager) : QMainWindow(parent) {
    setMinimumWidth(800);
    setMinimumHeight(600);

    // layouts
    mainLayout = new QVBoxLayout();
    horizontalLayout = new QHBoxLayout();
    vLayout = new QVBoxLayout();

    manualControlWidget = new ManualControlWidget(this);

    // add the tab widget
    auto tabWidget = new QTabWidget;




    auto SettingsTabWidget = new QTabWidget;
    tabWidget->addTab(SettingsTabWidget, tr("Settings"));
    tabWidget->addTab(manualControlWidget, tr("Manual"));

    vLayout->addWidget(tabWidget);

    // set up the general layout structure
    // visualizer on the left, sidebar (with maincontrols and tabs) on the right.
    auto splitter = new QSplitter();  // the splitter is an horizontal view that allows to be changed by the user
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

    robotsTimer->start(500);  // 2fps

    auto *graphTimer = new QTimer(this);
    graphTimer->start(500);  // 2fps


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

}  // namespace rtt::ai::interface
