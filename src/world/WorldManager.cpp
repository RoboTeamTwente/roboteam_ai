#include <roboteam_ai/src/utilities/Constants.h>
#include "WorldManager.h"
#include "../io/IOManager.h"

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

} // world
} // ai
} // rtt