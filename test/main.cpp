//
// Created by mrlukasbos on 19-9-18.
//

#include <include/roboteam_ai/utilities/Settings.h>
#include <include/roboteam_ai/world_new/World.hpp>

#include <gtest/gtest.h>
#include <utilities/Constants.h>
#include <utilities/RobotDealer.h>
#include <QtWidgets/QApplication>
#include <roboteam_proto/World.pb.h>


int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    QApplication app(argc, argv);  // initialize qt5
    rtt::ai::Constants::init();
    rtt::SETTINGS.init(0);
    rtt::ai::robotDealer::RobotDealer::setKeeperID(-1);

    return RUN_ALL_TESTS();
}
