#include <gtest/gtest.h>
#include <include/roboteam_ai/utilities/Settings.h>
#include <utilities/Constants.h>
#include <utilities/RobotDealer.h>
#include <QtWidgets/QApplication>

#define RUNNING_TEST

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    QApplication app(argc, argv);  // initialize qt5
    rtt::ai::Constants::init();
    rtt::SETTINGS.init(0);
    rtt::ai::robotDealer::RobotDealer::setKeeperID(-1);

    return RUN_ALL_TESTS();
}
