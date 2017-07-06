#pragma once

#include "ros/ros.h"

#include "std_srvs/Empty.h"

#include "roboteam_msgs/DetectionFrame.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/Tracker.h"

#include "roboteam_world/world/world_dummy.h"
#include "roboteam_world/world/filtered_world.h"
#include "roboteam_world/tracker/tracker.h"
#include "roboteam_world/tracker/acceleration_tracker.h"
#include "roboteam_world/tracker/speed_tracker.h"


namespace rtt {

    class RosHandler {

    private:
        ros::NodeHandle nh;
        ros::Subscriber vision_sub;
        ros::Publisher world_pub;
        ros::ServiceServer reset_srv;
        ros::ServiceServer tracker_srv;

        WorldBase* world;
        Tracker tracker;

    public:
        RosHandler();
        void init(WorldBase* _world);

        /**
         * Reads the configuration from the parameter server.
         * Updates the configuration of the world and calls a reset.
         */
        void update_config();

        void detection_callback(const roboteam_msgs::DetectionFrame msg);
        bool reset_callback(std_srvs::Empty::Request& req,
                            std_srvs::Empty::Response& res);

        bool tracker_callback(roboteam_msgs::Tracker::Request& req,
                              roboteam_msgs::Tracker::Response& res);

        /**
         * @brief Attempts to compute a tracking result for a certain robot.
         * @param type The name of the tracker which should have the data.
         * @param id The robot to track.
         * @return The result of the tracking if the module exists, nullptr otherwise.
         * Note that even if a TrackerResult is returned, that does not mean the calculation
         * succeeded. Always check the success flag.
         */
        TrackerResult* track(const std::string& type, const TeamRobot& bot) const;
    };

}
