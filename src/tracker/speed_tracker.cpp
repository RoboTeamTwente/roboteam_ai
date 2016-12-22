#include "roboteam_world/tracker/speed_tracker.h"

namespace rtt {
    
const std::string SpeedTracker::name() const {
    return TRACKER_TYPE_SPEED;
}    

Position* SpeedTracker::calculate(const RobotID& id) const {
    unsigned int max_idx = std::min((const unsigned int) get_sample_count(), num_samples);
    Position accumulator;
    Position last_pos;
    for (unsigned int i = 0; i < max_idx; i++) {
        Position pos = samples.at(id)[i];
        if (i > 0) {
            accumulator = accumulator + (pos - last_pos);
        }
        last_pos = pos;
    }
    return new Position(accumulator * (1.0/(double)max_idx));
}    
    
}