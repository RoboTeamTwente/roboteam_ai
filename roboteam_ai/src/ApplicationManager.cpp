//
// Created by mrlukasbos on 14-1-19.
//

#include "ApplicationManager.h"
#include <sstream>
#include <roboteam_ai/src/interface/api/Output.h>
#include <roboteam_ai/src/interface/api/Input.h>
#include <roboteam_ai/src/interface/api/Toggles.h>

namespace io = rtt::ai::io;
namespace ai = rtt::ai;

namespace rtt {

void ApplicationManager::setup() {
    IOManager = new io::IOManager(true, false);
    tc = new ai::BallPlacementWithInterface();
}

void ApplicationManager::loop() {
    ros::Rate rate(ai::Constants::TICK_RATE());

    while (ros::ok()) {
        this->runOneLoopCycle();
        rate.sleep();
    }
}

void ApplicationManager::runOneLoopCycle() {
    if (weHaveRobots) {
        tc->onUpdate();
    }
    else {
        std::cout << "NO FIRST WORLD" << std::endl;
        ros::Duration(0.2).sleep();
    }

    weHaveRobots = ai::world::world->weHaveRobots();
}

void ApplicationManager::checkForShutdown() {
    // Terminate if needed
}

} //rtt
