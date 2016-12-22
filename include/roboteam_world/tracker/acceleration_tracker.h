#pragma once

#include "position_based_tracker.h"

namespace rtt {
   
class AccelerationTracker : public PositionBasedTracker {
public:    
    const std::string name() const override;
    Position* calculate(const RobotID& id) const;
};

}