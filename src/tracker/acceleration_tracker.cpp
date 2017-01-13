#include "roboteam_world/tracker/acceleration_tracker.h"

namespace rtt {

const std::string AccelerationTracker::name() const {
    return TRACKER_TYPE_ACCEL;
}    

Position* AccelerationTracker::calculate(const TeamRobot& bot) const {
    unsigned int max_idx = std::min((const unsigned int) get_sample_count(), num_samples);
    Position accumulator;
    Position last_pos;
    Position last_vel;
    ros::Duration duration = samples.at(bot)[max_idx - 1].second - samples.at(bot)[0].second;
    for (unsigned int i = 0; i < max_idx; i++) {
        Position pos = samples.at(bot)[i].first;
        if (i > 0) {
            Position vel = pos - last_pos;
            if (i > 1) {
                accumulator = accumulator + (vel - last_vel);
            }
            last_vel = vel;
        }
        last_pos = pos;
    }
    return new Position(accumulator * (1.0/(double) max_idx) * (1.0/duration.toSec()));
}    
    
}