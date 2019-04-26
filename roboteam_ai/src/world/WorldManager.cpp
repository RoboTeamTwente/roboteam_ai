//
// Created by thijs on 21-3-19.
//

#include "WorldManager.h"
#include <roboteam_ai/src/utilities/StrategyManager.h>
#include <roboteam_ai/src/interface/InterfaceValues.h>

namespace rtt {
namespace ai {
namespace world {

void WorldManager::setup() {
    IOManager = new io::IOManager(true, false);
}

// spin ROS at 4x our tick rate
void WorldManager::loop() {
    ros::Rate rate(4.0 * ai::Constants::TICK_RATE());
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}


}
}
}