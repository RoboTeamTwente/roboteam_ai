#include <include/roboteam_ai/utilities/Settings.h>
#include <gtest/gtest.h>
#include <utilities/Constants.h>
#include <utilities/RobotDealer.h>
#include <QtWidgets/QApplication>

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    QApplication app(argc, argv);  // initialize qt5
    rtt::ai::Constants::init();
    rtt::SETTINGS.init(0);
    rtt::ai::robotDealer::RobotDealer::setKeeperID(-1);

    return RUN_ALL_TESTS();
}
