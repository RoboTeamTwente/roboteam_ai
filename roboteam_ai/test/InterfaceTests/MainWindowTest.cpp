//
// Created by mrlukasbos on 28-1-19.
//

#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>

namespace rtt {
namespace ai {
namespace interface {

TEST(MainWindowTest, it_displays_main_window) {
// set arguments to 0
int argc = 0;
char **argv = 0;

auto app = new QApplication(argc, argv);
auto window = new QMainWindow();
}

}
}
}