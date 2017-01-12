#include "roboteam_world/tracker/position_based_tracker.h"
#include "roboteam_msgs/WorldRobot.h"
#include <stdio.h>

namespace rtt {
    
void PositionBasedTracker::update(const World& world) {
    std::vector<roboteam_msgs::WorldRobot> bots = world.them;
    for (const roboteam_msgs::WorldRobot& bot : bots) {
        add_sample(bot.id, Position(bot.pos.x, bot.pos.y, bot.angle));
    }
    counter = (counter + 1) % num_samples;
    total_samples++;
}

unsigned long PositionBasedTracker::get_sample_count() const {
    return total_samples;
}

void PositionBasedTracker::add_sample(const RobotID& id, const Position& pos) {
    samples[id][counter] = {pos, ros::Time::now()};
}

TrackerResult PositionBasedTracker::calculate_for(const RobotID& id) const {
    TrackerResult res;
    res.type = TrackedValueType::VEC3;
    Position* pos = calculate(id);
    if (pos) {
        res.value = *pos;
        res.success = true;
        delete pos;
    } else {
        res.value = Position();
        res.success = false;
    }
    return res;
}

}