#include <utilities/IOManager.h>
#include <roboteam_utils/Print.h>
#include <include/roboteam_ai/world/World.hpp>
#include "ApplicationManager.h"
#include <QStyleFactory>
#include <QApplication>
namespace ui = rtt::ai::interface;

ui::MainWindow* window;

void run_application(int ai_id) {
    rtt::ApplicationManager app{ window };
    app.start(ai_id);
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

    std::cout << "                                           \n"
                 "  ██████╗ ████████╗████████╗     █████╗ ██╗\n"
                 "  ██╔══██╗╚══██╔══╝╚══██╔══╝    ██╔══██╗██║\n"
                 "  ██████╔╝   ██║      ██║       ███████║██║\n"
                 "  ██╔══██╗   ██║      ██║       ██╔══██║██║\n"
                 "  ██║  ██║   ██║      ██║       ██║  ██║██║\n"
                 "  ╚═╝  ╚═╝   ╚═╝      ╚═╝       ╚═╝  ╚═╝╚═╝\n"
                 "                                         " << std::endl;

    RTT_DEBUG("Debug prints enabled")



    // get the id of the ai from the init
    int id = 0;
    if (argc == 2) {
        id = *argv[1] - '0';
    }

    // initialize the interface
    QApplication a(argc, argv);
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    setDarkTheme();
  std::thread app_thread = std::thread(&run_application,id);

    // Todo make this a not-global-static thingy
    window = new ui::MainWindow{ };
    window->setWindowState(Qt::WindowMaximized);


    window->show();
    bool result = a.exec();
    return result;
}
