#include "roboteam_world/ros_handler.h"
#include "roboteam_utils/constants.h"

namespace rtt {

    RosHandler::RosHandler() {

    }


    void RosHandler::init(rtt::WorldBase* _world) {
        world = _world;

        // Subscribe to the vision input.
        vision_sub = nh.subscribe(TOPIC_DETECTION, 1000, &RosHandler::detection_callback, this);

        // Advertise the world output.
        world_pub = nh.advertise<roboteam_msgs::World>(TOPIC_WOLRD_STATE, 1000);

        // Advertise the reset service.
        reset_srv = nh.advertiseService(SERVICE_WORLD_RESET, &RosHandler::reset_callback, this);
    }


    void RosHandler::detection_callback(const roboteam_msgs::DetectionFrame msg) {
        world->detection_callback(msg);

        roboteam_msgs::World world_msg = world->as_message();

        world_pub.publish(world_msg);
    }


    bool RosHandler::reset_callback(
            std_srvs::Empty::Request& req,
            std_srvs::Empty::Response& res) {

        world->reset();

        return true;
    }

}
