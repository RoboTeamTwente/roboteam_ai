#pragma once

#include "roboteam_world/tracker/tracker.h"
#include "roboteam_world/tracker/tracker_utils.h"
#include <utility>

namespace rtt {

/**
 * The maximum amount of samples which will be used in the calculations of PositionBasedTrackers.
 * Higher numbers cause calculations to be more accurate, but slower.
 */
constexpr unsigned int num_samples = 10; 
   
/**
 * @class PositionBasedTracker
 * @author Dennis
 * @date 09/01/17
 * @file position_based_tracker.h
 * @brief Base class for trackers which do some calculation based on a robot's movement.
 */
class PositionBasedTracker : public TrackerModule {
public:    
    PositionBasedTracker() : counter(0), total_samples(0) {}
    virtual ~PositionBasedTracker() {}
    virtual void update(const World& world);
    virtual TrackerResult calculate_for(const TeamRobot& bot) const final override;
    
    /**
     * @brief Performs the specific calculation for the module.
     * @param id The robot to do the calculation for.
     * @return The result of the calculation.
     */
    virtual Position* calculate(const TeamRobot& bot) const = 0;
    
    /**
     * @brief Gets the total amount of world states this tracker has processed.
     * @return The count.
     */
    unsigned long get_sample_count() const;
    
protected:

    /**
     * @brief Add a new sample to the map.
     * @param id The robot the sample corresponds to.
     * @param pos The current position of the robot.
     */
    void add_sample(const TeamRobot& bot, const Position& pos);
    
    std::map<TeamRobot, std::array<std::pair<Position, ros::Time>, num_samples>> samples;
    unsigned int counter;
    unsigned long total_samples;
}; 
    
}