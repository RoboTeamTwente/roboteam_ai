//
// Created by maxl on 11-02-21.
//

#ifndef RTT_GOALCOMPUTATIONS_H
#define RTT_GOALCOMPUTATIONS_H

#include <roboteam_utils/LineSegment.h>
#include <world/Field.h>

#include <world/World.hpp>

namespace rtt::ai::stp::computations {

class GoalComputations {
   public:
    /**
     * Calculate point in goal to aim for
     * @return Target point
     */
    static Vector2 calculateGoalTarget(rtt::world::World *world, const rtt_world::Field &field);

    /**
     * Calculate points we want to aim for
     * @param field Field
     * @param fromPoint Position to shoot from
     * @return Line between the two aim points
     */
    static rtt::LineSegment getAimPoints(const rtt::world::Field &field, const rtt::Vector2 &sourcePoint);

    /**
     * Returns the longest line from openSegments
     * @param openSegments Vector of lines
     * @return Longest line from openSegments
     */
    static const LineSegment &getLongestSegment(const std::vector<LineSegment> &openSegments);
};
}  // namespace rtt::ai::stp::computations
#endif  // RTT_GOALCOMPUTATIONS_H
