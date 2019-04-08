//
// Created by robzelluf on 3/21/19.
//

#ifndef ROBOTEAM_AI_OFFENSIVECOACH_H
#define ROBOTEAM_AI_OFFENSIVECOACH_H

#include <roboteam_utils/Vector2.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include <roboteam_ai/src/world/Field.h>
#include <algorithm>
#include "CoachHeuristics.h"
#include "../world/WorldData.h"

namespace rtt {
namespace ai {
namespace coach {

class OffensiveCoach {
public:
    using Robot = world::Robot;
    using RobotPtr = std::shared_ptr<Robot>;

    const double SEARCH_GRID_ROBOT_POSITIONS = 0.02;
    const int GRID_SIZE = 3;
    const double CLOSE_TO_GOAL_DISTANCE = 0.95;
    const double FURTHER_FROM_GOAL_DISTANCE = 2 * CLOSE_TO_GOAL_DISTANCE;
    const double ZONE_RADIUS = 0.8;

    struct OffensivePosition {
        Vector2 position;
        double score;
    };

    OffensivePosition calculateNewRobotPosition(const OffensivePosition& currentPosition, const Vector2& defaultPosition);

    std::vector<Vector2> getOffensivePositionVectors();
    static int getBestStrikerID();

    std::vector<Vector2> getDefaultLocations();
    std::vector<Vector2> getNewOffensivePositions(int numberOfRobots);

private:
    std::vector<OffensivePosition> offensivePositions;
    std::map<int, OffensivePosition> robotPositions;

};

extern OffensiveCoach g_offensiveCoach;

}
}
}

#endif //ROBOTEAM_AI_OFFENSIVECOACH_H
