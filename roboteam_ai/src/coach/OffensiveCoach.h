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
        const double ATTACKER_DISTANCE = 1.6;
        const double OFFENSIVE_POSITION_DISTANCE = 0.8;
        const double SEARCH_GRID_ROBOT_POSITIONS = 0.01;
        const int GRID_SIZE = 2;

        struct OffensivePosition {
          Vector2 position;
          double score;
        };

        void calculateNewPositions();
        void calculateNewRobotPositions(const RobotPtr &robot);
        Vector2 calculatePositionForRobot(const RobotPtr &robot);
        void releaseRobot(int robotID);
        Vector2 getPositionForRobotID(int robotID);
        std::vector<OffensivePosition> getRobotPositionVectors();
        int getBestStrikerID();
        const vector<OffensivePosition> &getOffensivePositions();
        const map<int, OffensivePosition> &getRobotPositions();

    private:

        double marginFromLines = 0.2;

        std::vector<OffensivePosition> offensivePositions;
        int maxPositions = 4;
        std::map<int, OffensivePosition> robotPositions;

        static bool compareByScore(OffensivePosition position1, OffensivePosition position2);
        void drawOffensivePoints();
        void recalculateOffensivePositions();
        OffensivePosition calculateRandomPosition(double xStart, double xEnd, double yStart, double yEnd);
        bool positionTooCloseToRobotPositions(OffensivePosition position, int self = - 1);

        void compareToCurrentPositions(const OffensivePosition &position);

        Vector2 getClosestOffensivePosition(const RobotPtr &robot);
};

extern OffensiveCoach g_offensiveCoach;

}
}
}

#endif //ROBOTEAM_AI_OFFENSIVECOACH_H
