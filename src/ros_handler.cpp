#include "roboteam_world/ros_handler.h"
#include "roboteam_utils/constants.h"

namespace rtt {

    RosHandler::RosHandler() {

    }

    bool cool_bool = false;

    void RosHandler::init(rtt::WorldBase* _world) {
        world = _world;

        // Subscribe to the vision input.
        vision_sub = nh.subscribe(TOPIC_DETECTION, 1000, &RosHandler::detection_callback, this);

        // Advertise the world output.
        world_pub = nh.advertise<roboteam_msgs::World>(TOPIC_WORLD_STATE, 1);

        // Advertise the reset service.
        reset_srv = nh.advertiseService(SERVICE_WORLD_RESET, &RosHandler::reset_callback, this);
        
        // Advertise the tracker service.
        tracker_srv = nh.advertiseService(SERVICE_OPPONENT_TRACKER, &RosHandler::tracker_callback, this);
        tracker.add_module(new SpeedTracker());
        tracker.add_module(new AccelerationTracker());
    }


    void RosHandler::detection_callback(const roboteam_msgs::DetectionFrame msg) {
        world->detection_callback(msg);

        if (FilteredWorld* fworld = dynamic_cast<FilteredWorld*>(world)) {
            // Filtered world! Special case that shit
            if (auto worldOpt = fworld->consumeMsg()) {
                world_pub.publish(*worldOpt);
                tracker.update(*worldOpt);
            }
        } else {
            // Generic world approach
            // Where you can never be sure if this message is the new one
            roboteam_msgs::World world_msg = world->as_message();

            world_pub.publish(world_msg);
            tracker.update(world_msg);
        }
    }

    bool RosHandler::reset_callback(
            std_srvs::Empty::Request& req,
            std_srvs::Empty::Response& res) {

        world->reset();

        return true;
    }
    
    bool RosHandler::tracker_callback(roboteam_msgs::Tracker::Request& req, roboteam_msgs::Tracker::Response& res) {
        
        const TrackerModule* module = tracker.get_module(req.type);
        if (module == nullptr) {
            res.success = false;
            return false;
        }
        
        TrackerResult track_result = module->calculate_for({req.id, req.our_team});
        res.success = track_result.success;
        switch (track_result.type) {
            case TrackedValueType::LONG: res.long_val = boost::get<long>(track_result.value); break;
            case TrackedValueType::DOUBLE: res.double_val = boost::get<double>(track_result.value); break;
            case TrackedValueType::BOOL: res.bool_val = boost::get<bool>(track_result.value); break;
            case TrackedValueType::STRING: res.string_val = boost::get<std::string>(track_result.value); break;
            case TrackedValueType::VEC3:
                Position pos = boost::get<Position>(track_result.value);
                res.vector_val.x = pos.x;
                res.vector_val.y = pos.y;
                res.vector_val.z = pos.rot; 
                break;
        }
        
        return true;
    }

    TrackerResult* RosHandler::track(const std::string& type, const TeamRobot& bot) const {
        const TrackerModule* module = tracker.get_module(type);
        return module == nullptr ? nullptr : new TrackerResult(module->calculate_for(bot));
    }

}
