#pragma once

#include "opponent_tracker.h"

namespace rtt {

constexpr unsigned int num_samples = 5; 
   
class PositionBasedTracker : public TrackerModule {
public:    
    PositionBasedTracker() : counter(0), total_samples(0) {}
    virtual ~PositionBasedTracker() {}
    virtual void update(const World& world) override;
    virtual TrackerResult calculate_for(const RobotID& id) const final override;
    
    virtual Position* calculate(const RobotID& id) const = 0;
    unsigned long get_sample_count() const;
protected:    
    void add_sample(const RobotID& id, const Position& pos);
    std::map<RobotID, std::array<Position, num_samples>> samples;
    unsigned int counter;
    unsigned long total_samples;
}; 
    
}