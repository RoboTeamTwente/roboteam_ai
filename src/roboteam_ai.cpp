#include <roboteam_utils/Print.h>

#include "STPManager.h"
#include "utilities/IOManager.h"
#include "world/World.hpp"

namespace ui = rtt::ai::interface;

ui::MainWindow* window;

void runStp() {
    rtt::STPManager app{window};
    app.start();
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

int main(int argc, char* argv[]) {
    if (argc != 2) {
        RTT_ERROR("Incorrect amount of arguments")
        RTT_INFO("Pass '0' as argument to indicate this is the primary AI, or anything else for a secondary AI")
        return 0;
    }

    RTT_INFO("\n",
             "                                           \n"
             "  ██████╗ ████████╗████████╗     █████╗ ██╗\n"
             "  ██╔══██╗╚══██╔══╝╚══██╔══╝    ██╔══██╗██║\n"
             "  ██████╔╝   ██║      ██║       ███████║██║\n"
             "  ██╔══██╗   ██║      ██║       ██╔══██║██║\n"
             "  ██║  ██║   ██║      ██║       ██║  ██║██║\n"
             "  ╚═╝  ╚═╝   ╚═╝      ╚═╝       ╚═╝  ╚═╝╚═╝\n"
             "                                           ")

    RTT_DEBUG("Debug prints enabled")

    rtt::ai::Constants::init();

    // get the id of the ai from the init
    int id = *argv[1] - '0';

    rtt::SETTINGS.init(id);

    // If primary AI, we start at being yellow on the left
    if (!rtt::SETTINGS.setYellow(rtt::SETTINGS.isPrimaryAI())) {
        RTT_ERROR("Could not obtain command publishing channel. Exiting...")
        return 0;
    }

    rtt::SETTINGS.setLeft(rtt::SETTINGS.isPrimaryAI());

    rtt::SETTINGS.setRobotHubMode(rtt::Settings::RobotHubMode::BASESTATION);
    rtt::SETTINGS.setVisionIp("127.0.0.1");
    rtt::SETTINGS.setVisionPort(10006);
    rtt::SETTINGS.setRefereeIp("224.5.23.1");
    rtt::SETTINGS.setRefereePort(10003);
    rtt::SETTINGS.setRobothubSendIp("127.0.0.1");
    rtt::SETTINGS.setRobothubSendPort(20011);

    RTT_INFO("AI initialized as: ", (rtt::SETTINGS.isPrimaryAI() ? "PRIMARY" : "SECONDARY"))
    RTT_INFO("Starting as color: ", (rtt::SETTINGS.isYellow() ? "YELLOW" : "BLUE"))
    RTT_INFO("Playing on side: ", (rtt::SETTINGS.isLeft() ? "LEFT" : "RIGHT"))
    RTT_INFO("This AI will ", rtt::SETTINGS.isPrimaryAI() ? "" : "NOT ", "broadcast settings")

    if (!rtt::ai::io::io.init(rtt::SETTINGS.isPrimaryAI())) {
        RTT_ERROR("Failed to initialize IO Manager. Exiting...")
        return 0;
    }

    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    // initialize the interface
    QApplication a(argc, argv);
    setDarkTheme();

    // Todo make this a not-global-static thingy
    window = new ui::MainWindow{};
    window->setWindowState(Qt::WindowMaximized);

    std::thread stpThread = std::thread(&runStp);

    window->show();
    return a.exec();
}
