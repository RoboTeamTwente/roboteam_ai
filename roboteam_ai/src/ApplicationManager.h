//
// Created by mrlukasbos on 14-1-19.
//

#ifndef ROBOTEAM_AI_APPLICATIONMANAGER_H
#define ROBOTEAM_AI_APPLICATIONMANAGER_H

#include <gtest/gtest_prod.h>
#include "io/IOManager.h"
#include "ros/ros.h"
#include "skills/BallPlacementWithInterface.h"

namespace rtt {

class ApplicationManager {
private:
    FRIEND_TEST(ApplicationManagerTest, it_handles_ROS_data);
    rtt::ai::io::IOManager * IOManager;

    ai::BallPlacementWithInterface * tc;


    void runOneLoopCycle();
    bool weHaveRobots = false;
public:
    void setup();
    void loop();
    void checkForShutdown();
};

} // rtt

#endif //ROBOTEAM_AI_APPLICATIONMANAGER_H
