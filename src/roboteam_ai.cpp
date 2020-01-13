#include <QApplication>
#include <QStyleFactory>
#include <utilities/Constants.h>
#include "include/roboteam_ai/settings/settings.hpp"

#include "interface/widgets/mainWindow.h"
#include "ApplicationManager.h"

namespace ui = rtt::ai::interface;
std::shared_ptr<ui::MainWindow> window;

void runBehaviourTrees(rtt::Settings& settings) {
    rtt::ApplicationManager app;
    app.start(settings);
    app.checkForShutdown();
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
    rtt::ai::Constants::init();

    // get the id of the ai from the init
    int id = 0;
    if (argc == 2) {
        id = *argv[1] - '0';
    }

    rtt::Settings settings{ };
    rtt::Settings::settings = &settings;

    // some default settings for different team ids (saves time while testing)
    settings.init(id);
    settings.setYellow(id != 1);
    settings.setLeft(id != 1);
    settings.setSerialMode(false);
    settings.setVisionIp("127.0.0.1");
    settings.setVisionPort(10006);
    settings.setRefereeIp("224.5.23.1");
    settings.setRefereePort(10007);
    settings.setRobothubSendIp("127.0.0.1");
    settings.setRobothubSendPort(20011);

    auto io = rtt::ai::io::IOManager(settings);
    rtt::ai::io::io = &io;


    BTFactory::makeTrees();
    while (!BTFactory::hasMadeTrees());

    std::thread behaviourTreeThread = std::thread([&](){
        runBehaviourTrees(settings);
    });

    // initialize the interface
    QApplication a(argc, argv);
    setDarkTheme();
    window = std::make_shared<ui::MainWindow>(nullptr, settings);
    window->setWindowState(Qt::WindowMaximized);

    window->show();

    return a.exec();
}

