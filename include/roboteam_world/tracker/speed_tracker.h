#pragma once

#include "position_based_tracker.h"

namespace rtt {
   
class SpeedTracker : public PositionBasedTracker {
public:
    const std::string name() const override;
    Position* calculate(const RobotID& id) const;
}; 
    
}