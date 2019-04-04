//
// Created by mrlukasbos on 19-9-18.
//

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <QtWidgets/QApplication>
#include <roboteam_ai/src/utilities/Constants.h>
#include <roboteam_ai/src/utilities/RobotDealer.h>

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    QApplication app(argc, argv); // initialize qt5

    // create a ROS node for the tests
    ros::init(argc, argv, "tester");
    rtt::ai::Constants::init();
    ros::NodeHandle nh;

    robotDealer::RobotDealer::setUseSeparateKeeper(false);


    return RUN_ALL_TESTS();
}
