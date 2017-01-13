#include "roboteam_world/tracker/speed_tracker.h"
#include <stdio.h>

namespace rtt {
    
const std::string SpeedTracker::name() const {
    return TRACKER_TYPE_SPEED;
}    

Position* SpeedTracker::calculate(const TeamRobot& bot) const {
    unsigned int max_idx = std::min((const unsigned int) get_sample_count(), num_samples);
    Position accumulator;
    Position last_pos;
    ros::Duration duration = samples.at(bot)[max_idx - 1].second - samples.at(bot)[0].second;
    for (unsigned int i = 0; i < max_idx; i++) {
        Position pos = samples.at(bot)[i].first;
        if (i > 0) {
            accumulator = accumulator + (pos - last_pos);
        }
        last_pos = pos;
    }
    return new Position(accumulator * (1.0/(double)max_idx) * (1.0/duration.toSec()));
}    
    
}