//
// Created by robzelluf on 3/21/19.
//

#ifndef ROBOTEAM_AI_OFFENSIVECOACH_H
#define ROBOTEAM_AI_OFFENSIVECOACH_H

#include <roboteam_utils/Vector2.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include <roboteam_ai/src/world/Field.h>
#include <algorithm>
#include "roboteam_ai/src/coach/heuristics/CoachHeuristics.h"
#include "../world/WorldData.h"
#include "heuristics/OffensiveScore.h"

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

    std::vector<Vector2> getDefaultLocations();
    void updateOffensivePositions();
    std::vector<Vector2> getOffensivePositions(int numberOfRobots);

    void addSideAttacker(const RobotPtr& robot);
    void removeSideAttacker(const RobotPtr& robot);
    Vector2 getPositionForRobotID(int robotID);
    void redistributePositions();

    Vector2 getShootPoint(const Vector2& fromPoint);
    Vector2 getShootPoint2(const Vector2& fromPoint);


    private:
    coach::OffensiveScore offensiveScore;
    std::vector<OffensivePosition> offensivePositions;
    std::map<int, int> sideAttackers; // Map from robot ids to zones

};

extern OffensiveCoach g_offensiveCoach;

}
}
}

#endif //ROBOTEAM_AI_OFFENSIVECOACH_H
