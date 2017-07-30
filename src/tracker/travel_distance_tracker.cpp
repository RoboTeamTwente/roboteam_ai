#include "roboteam_world/tracker/travel_distance_tracker.h"

namespace rtt {
 
const std::string TravelDistanceTracker::name() const {
    return "TravelDistance";
}

template <typename K, typename V>
inline bool has_bot(const std::map<K, V>& map, const K& key) {
    return map.find(key) != map.end();
}

void TravelDistanceTracker::update(const roboteam_msgs::WorldRobot& bot) {
    RobotID id = bot.id;
    if (has_bot(last_positions, id)) {
        distances[id] = distances[id] + last_positions.at(id).dist(bot.pos);
    } else if (!has_bot(distances, id)) {
        distances[id] = 0;
    }
    last_positions[id] = bot.pos;
}

void TravelDistanceTracker::update(const World& world) {
    for (const auto bot : world.us) {
        update(bot);
    }
    for (const auto bot : world.them) {
        update(bot);
    }
}

TrackerResult TravelDistanceTracker::calculate_for(const TeamRobot& bot) const  {
    TrackerResult res;
    if (!has_bot(distances, bot.id)) {
        res.success = false;
    } else {
        res.success = true;
        res.type = TrackedValueType::DOUBLE;
        res.value = distances.at(bot.id);
    }
    return res;
}
    
}