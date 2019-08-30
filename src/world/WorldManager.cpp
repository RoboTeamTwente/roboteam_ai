#include <include/roboteam_ai/utilities/Constants.h>
#include "include/roboteam_ai/world/WorldManager.h"
#include "include/roboteam_ai/io/IOManager.h"

namespace rtt {
namespace ai {
namespace world {

void WorldManager::setup() {
}

// spin ROS at 4x our tick rate
void WorldManager::loop() {
//    ros::Rate rate(4.0 * ai::Constants::TICK_RATE());
//    while (ros::ok()) {
//        ros::spinOnce();
//        rate.sleep();
//    }
}

} // world
} // ai
} // rtt