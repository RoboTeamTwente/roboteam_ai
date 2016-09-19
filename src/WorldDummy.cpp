#include "WorldDummy.h"

#include <iostream>


WorldDummy::WorldDummy() {

}


void WorldDummy::detectionCallback(const roboteam_vision::DetectionFrame msg) {
    ROS_INFO("From [%u]\n[%lu]", msg.camera_id, msg.robots_yellow.size());

    std::vector<roboteam_vision::DetectionRobot> robots_yellow = msg.robots_yellow;

    for (std::vector<roboteam_vision::DetectionRobot>::iterator it =
        robots_yellow.begin(); it != robots_yellow.end(); ++it)
    {
        std::cout << it->robot_id << ": " << it->x << "," << it->y << std::endl;
    }
}
