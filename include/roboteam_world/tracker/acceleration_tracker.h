#pragma once

#include "position_based_tracker.h"

namespace rtt {
   
/**
 * @class AccelerationTracker
 * @author Dennis
 * @date 09/01/17
 * @file acceleration_tracker.h
 * @brief Track robots' acceleration.
 */
class AccelerationTracker : public PositionBasedTracker {
public:    
    const std::string name() const override;
    Position* calculate(const TeamRobot& id) const;
};

}