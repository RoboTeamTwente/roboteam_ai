//
// Created by mrlukasbos on 19-9-18.
//

#include <gtest/gtest.h>
#include <QtWidgets/QApplication>
#include <roboteam_ai/utilities/Constants.h>
#include <roboteam_ai/utilities/RobotDealer.h>
#include <roboteam_ai/Settings/Settings.h>

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    QApplication app(argc, argv); // initialize qt5
    rtt::ai::Constants::init();
    rtt::settings.init();
    rtt::ai::robotDealer::RobotDealer::setKeeperID(-1);

    return RUN_ALL_TESTS();
}
