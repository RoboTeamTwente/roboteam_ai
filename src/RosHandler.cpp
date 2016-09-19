#include "RosHandler.h"


RosHandler::RosHandler() {

}


void RosHandler::init(WorldDummy* _world) {
    world = _world;

    // Subscribe to the vision input.
    vision_sub = n.subscribe("vision_detection", 1000, &WorldDummy::detectionCallback, world);

    // Advertise the world output.
    world_pub = n.advertise<roboteam_world::World>("world_state", 1000);
}
