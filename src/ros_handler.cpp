#include "roboteam_world/ros_handler.h"


namespace rtt {

    RosHandler::RosHandler() {

    }


    void RosHandler::init(rtt::WorldBase* _world) {
        world = _world;

        // Subscribe to the vision input.
        vision_sub = nh.subscribe("vision_detection", 1000, &RosHandler::detection_callback, this);

        // Advertise the world output.
        world_pub = nh.advertise<roboteam_msgs::World>("world_state", 1000);

        // Advertise the reset service.
        reset_srv = nh.advertiseService("world_reset", &RosHandler::reset_callback, this);

        // Initialize parameters.
        update_config();
    }


    void RosHandler::update_config() {
        WorldConfig config = world->get_config();

        ROS_INFO("---- World configuration ----");

        int num_cams;
        if (nh.getParam("w_num_cams", num_cams)) {
            config.set_num_cams(num_cams);
        } else {
            num_cams = config.num_cams();
            nh.setParam("w_num_cams", num_cams);
        }
        ROS_INFO("num_cams: %i", num_cams);

        // Reset the world with the new configuration.
        world->reset(config);
    }


    void RosHandler::detection_callback(const roboteam_msgs::DetectionFrame msg) {
        world->detection_callback(msg);

        roboteam_msgs::World world_msg = world->as_message();

        world_pub.publish(world_msg);
    }


    bool RosHandler::reset_callback(
            std_srvs::Empty::Request& req,
            std_srvs::Empty::Response& res) {

        update_config();

        return true;
    }

}
