#include "ros_handler.h"


RosHandler::RosHandler() {

}


void RosHandler::init(rtt::WorldBase* _world) {
    world = _world;

    // Subscribe to the vision input.
    vision_sub = n.subscribe("vision_detection", 1000, &RosHandler::detection_callback, this);

    // Advertise the world output.
    world_pub = n.advertise<roboteam_world::World>("world_state", 1000);
}


void RosHandler::detection_callback(const roboteam_vision::DetectionFrame msg) {
    world->detection_callback(msg);

    roboteam_world::World world_msg = world->as_message();

    world_pub.publish(world_msg);
}
