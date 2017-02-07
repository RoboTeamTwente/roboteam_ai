#pragma once

#include "roboteam_world/tracker/tracker.h"
#include "roboteam_world/tracker/tracker_utils.h"
#include "roboteam_msgs/WorldRobot.h"
#include <map>

namespace rtt {

class TravelDistanceTracker: public CountingTrackerBase {
    
public:
    const std::string name() const override;
    virtual void update(const World& world) override;
    virtual TrackerResult calculate_for(const TeamRobot& bot) const override;
    
private:
    std::map<RobotID, double> distances;
    std::map<RobotID, Vector2> last_positions;
    
    void update(const roboteam_msgs::WorldRobot& bot);
};
    
}