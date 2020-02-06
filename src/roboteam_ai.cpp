#include <include/roboteam_ai/utilities/Settings.h>
#include <utilities/Constants.h>

#include <QApplication>
#include <QStyleFactory>

#include "ApplicationManager.h"
#include "interface/widgets/mainWindow.h"

namespace ui = rtt::ai::interface;
std::shared_ptr<ui::MainWindow> window;

void runBehaviourTrees() {
    rtt::ApplicationManager app;
    app.start();
    app.checkForShutdown();
}

void setDarkTheme() {
    qApp->setStyle(QStyleFactory::create("Fusion"));
    QPalette darkPalette;
    darkPalette.setColor(QPalette::Window, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::WindowText, Qt::white);
    darkPalette.setColor(QPalette::Base, QColor(25, 25, 25));
    darkPalette.setColor(QPalette::AlternateBase, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ToolTipBase, Qt::white);
    darkPalette.setColor(QPalette::ToolTipText, Qt::white);
    darkPalette.setColor(QPalette::Text, Qt::white);
    darkPalette.setColor(QPalette::Button, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ButtonText, Qt::white);
    darkPalette.setColor(QPalette::BrightText, Qt::red);
    darkPalette.setColor(QPalette::Link, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::Highlight, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::HighlightedText, Qt::black);
    qApp->setPalette(darkPalette);
    qApp->setStyleSheet("QToolTip { color: #ffffff; background-color: #2a82da; border: 1px solid white; }");
}

int main(int argc, char *argv[]) {
    rtt::ai::Constants::init();

    // get the id of the ai from the init
    int id = 0;
    if (argc == 2) {
        id = *argv[1] - '0';
    }

    // some default settings for different team ids (saves time while testing)
    if (id == 1) {
        // standard blue team on right
        rtt::SETTINGS.init(id);
        rtt::SETTINGS.setYellow(false);
        rtt::SETTINGS.setLeft(false);
    } else {
        // standard yellow team on left
        rtt::SETTINGS.init(id);
        rtt::SETTINGS.setYellow(true);
        rtt::SETTINGS.setLeft(true);
    }

    rtt::SETTINGS.setSerialMode(false);
    rtt::SETTINGS.setVisionIp("127.0.0.1");
    rtt::SETTINGS.setVisionPort(10006);
    rtt::SETTINGS.setRefereeIp("224.5.23.1");
    rtt::SETTINGS.setRefereePort(10007);
    rtt::SETTINGS.setRobothubSendIp("127.0.0.1");
    rtt::SETTINGS.setRobothubSendPort(20011);

    rtt::ai::io::io.init();

    BTFactory::makeTrees();
    while (!BTFactory::hasMadeTrees())
        ;

    std::thread behaviourTreeThread = std::thread(&runBehaviourTrees);

    // initialize the interface
    QApplication a(argc, argv);
    setDarkTheme();
    window = std::make_shared<ui::MainWindow>();
    window->setWindowState(Qt::WindowMaximized);

    window->show();

    return a.exec();
}
