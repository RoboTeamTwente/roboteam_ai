#pragma once

#include "position_based_tracker.h"

namespace rtt {
   
/**
 * @class SpeedTracker
 * @author Dennis
 * @date 09/01/17
 * @file speed_tracker.h
 * @brief Tracks robots' speed.
 */
class SpeedTracker : public PositionBasedTracker {
public:
    const std::string name() const override;
    Position* calculate(const RobotID& id) const;
}; 
    
}