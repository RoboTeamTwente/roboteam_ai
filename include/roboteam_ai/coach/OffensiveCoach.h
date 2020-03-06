//
// Created by robzelluf on 3/21/19.
//

#ifndef ROBOTEAM_AI_OFFENSIVECOACH_H
#define ROBOTEAM_AI_OFFENSIVECOACH_H

#include <roboteam_utils/Hungarian.h>
#include <roboteam_utils/Line.h>
#include <roboteam_utils/Vector2.h>

#include <algorithm>
#include <include/roboteam_ai/world_new/views/RobotView.hpp>
#include "coach/heuristics/OffensiveScore.h"

namespace rtt::ai::coach {
using namespace rtt::ai::world;

class OffensiveCoach {
   public:
    const double SEARCH_GRID_ROBOT_POSITIONS = 0.055;
    const double CLOSE_TO_GOAL_DISTANCE = 0.37;
    const double FURTHER_FROM_GOAL_DISTANCE = 6.0 * CLOSE_TO_GOAL_DISTANCE;
    const double ZONE_RADIUS = 1.06;

    struct OffensivePosition {
        Vector2 position;
        double score;
        OffensivePosition() = default;
        constexpr explicit OffensivePosition(const Vector2 &position, double score = 0.0) : position(position), score(score) {}
    };

    OffensivePosition calculateNewRobotPosition(const Field &field, const OffensivePosition &currentPosition, const Vector2 &zoneLocation, int &tick, Angle &targetAngle);

    std::vector<Vector2> getZoneLocations(const Field &field);
    void updateOffensivePositions(const Field &field);
    std::vector<Vector2> getOffensivePositions(int numberOfRobots);

    void addSideAttacker(const Field &field, const world_new::view::RobotView &robot);

    void removeSideAttacker(const world_new::view::RobotView &robot);

    Vector2 getPositionForRobotID(const Field &field, int robotID);
    void redistributePositions(const Field &field);

    Vector2 getShootAtGoalPoint(const Field &field, const Vector2 &fromPoint);

   private:
    OffensivePosition findBestOffensivePosition(const Field &field, const std::vector<Vector2> &positions, const OffensivePosition &currentBestScore, const Vector2 &zoneLocation);

    const Line &getLongestSegment(const std::vector<Line> &openSegments);
    Line getAimPoints(const Field &field, const Vector2 &fromPoint);
    coach::OffensiveScore offensiveScore;
    std::vector<OffensivePosition> offensivePositions;
    std::map<int, int> sideAttackers;  // Map from robot ids to zones
};

extern OffensiveCoach g_offensiveCoach;

}  // namespace rtt::ai::coach

#endif  // ROBOTEAM_AI_OFFENSIVECOACH_H
