#include <QApplication>
#include <QStyleFactory>

#include "roboteam_ai/src/interface/widgets/mainWindow.h"
#include "ApplicationManager.h"
#include "world/WorldManager.h"

namespace ui = rtt::ai::interface;
std::shared_ptr<ui::MainWindow> window;

void runBehaviourTrees() {
    rtt::ApplicationManager app;
    app.setup();
    app.loop();
    app.checkForShutdown();
}

void runWorld() {
    rtt::ai::world::WorldManager worldManager;
    worldManager.setup();
    worldManager.loop();
}

void setDarkTheme() {
    qApp->setStyle(QStyleFactory::create("Fusion"));
    QPalette darkPalette;
    darkPalette.setColor(QPalette::Window, QColor(53,53,53));
    darkPalette.setColor(QPalette::WindowText, Qt::white);
    darkPalette.setColor(QPalette::Base, QColor(25,25,25));
    darkPalette.setColor(QPalette::AlternateBase, QColor(53,53,53));
    darkPalette.setColor(QPalette::ToolTipBase, Qt::white);
    darkPalette.setColor(QPalette::ToolTipText, Qt::white);
    darkPalette.setColor(QPalette::Text, Qt::white);
    darkPalette.setColor(QPalette::Button, QColor(53,53,53));
    darkPalette.setColor(QPalette::ButtonText, Qt::white);
    darkPalette.setColor(QPalette::BrightText, Qt::red);
    darkPalette.setColor(QPalette::Link, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::Highlight, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::HighlightedText, Qt::black);
    qApp->setPalette(darkPalette);
    qApp->setStyleSheet("QToolTip { color: #ffffff; background-color: #2a82da; border: 1px solid white; }");
}

int main(int argc, char* argv[]) {
    // Init ROS node in main thread
    ros::init(argc, argv, "Roboteam_AI");
    rtt::ai::Constants::init();

    std::thread worldThread = std::thread(&runWorld);
    std::thread behaviourTreeThread = std::thread(&runBehaviourTrees);

    // initialize the interface
    QApplication a(argc, argv);
    setDarkTheme();
    window = std::make_shared<ui::MainWindow>();
    window->show();
     return a.exec();
}

