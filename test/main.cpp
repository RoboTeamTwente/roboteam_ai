#include <gtest/gtest.h>
#include <utilities/Constants.h>
#include <utilities/Settings.h>

#include <QtWidgets/QApplication>

#define RUNNING_TEST

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    QApplication app(argc, argv);  // initialize qt5
    rtt::ai::Constants::init();
    rtt::SETTINGS.init(0);

    return RUN_ALL_TESTS();
}
