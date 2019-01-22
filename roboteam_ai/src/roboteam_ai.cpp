#include "interface/mainWindow.h"
#include <QApplication>
#include "ApplicationManager.h"

namespace ui = rtt::ai::interface;
std::shared_ptr<ui::MainWindow> window;

void runBehaviourTrees() {
    rtt::ApplicationManager app;
    app.setup();
    app.loop();
    app.checkForShutdown();
}

int main(int argc, char* argv[]) {
    // Init ROS node in main thread
    ros::init(argc, argv, "Roboteam_AI");

    // start the ros loop in separate thread
    std::thread behaviourTreeThread = std::thread(&runBehaviourTrees);

    // initialize the interface
    QApplication a(argc, argv);
    window = std::make_shared<ui::MainWindow>();
    window->show();

    return a.exec();
}

