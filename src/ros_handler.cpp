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
        
        // Advertise the tracker service.
        tracker_srv = nh.advertiseService(SERVICE_OPPONENT_TRACKER, &RosHandler::tracker_callback, this);
        tracker.add_module(new SpeedTracker());
        tracker.add_module(new AccelerationTracker());
    }


    void RosHandler::detection_callback(const roboteam_msgs::DetectionFrame msg) {
        world->detection_callback(msg);

        roboteam_msgs::World world_msg = world->as_message();

        world_pub.publish(world_msg);
        tracker.update(world_msg);
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
        
        TrackerResult track_result = module->calculate_for(req.id);
        res.success = track_result.success;
        switch (track_result.type) {
            case TrackedValueType::LONG: res.long_val = track_result.value.long_val; break;
            case TrackedValueType::DOUBLE: res.double_val = track_result.value.double_val; break;
            case TrackedValueType::BOOL: res.bool_val = track_result.value.bool_val; break;
            case TrackedValueType::STRING: res.string_val = track_result.value.string_val; break;
            case TrackedValueType::VEC3: 
                res.vector_val.x = track_result.value.pos_val.x;
                res.vector_val.y = track_result.value.pos_val.y;
                res.vector_val.z = track_result.value.pos_val.rot; 
                break;
        }
        
        return true;
    }

    TrackerResult* RosHandler::track(const std::string& type, const RobotID& id) const {
        const TrackerModule* module = tracker.get_module(type);
        return module == nullptr ? nullptr : new TrackerResult(module->calculate_for(id));
    }

}
