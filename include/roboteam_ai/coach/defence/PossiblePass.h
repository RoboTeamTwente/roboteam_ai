//
// Created by rolf on 5-4-19.
//

#ifndef ROBOTEAM_AI_POSSIBLEPASS_H
#define ROBOTEAM_AI_POSSIBLEPASS_H

#include <roboteam_utils/Vector2.h>
#include <include/roboteam_ai/world/Field.h>
#include "utilities/Constants.h"
#include "world/Robot.h"

namespace rtt {
namespace ai {
namespace coach {

class PossiblePass {
    public:
        world::Robot toBot;
        Vector2 startPos;
        Vector2 endPos;
        const double distance();
        bool obstacleObstructsPath(const Vector2 &obstPos,
                double obstRadius = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
        int amountOfBlockers(const world::WorldData &world);
        PossiblePass(world::Robot _toBot, const Vector2 &ballPos);
        double score(const Field &field, const world::WorldData &world);
        // scale from startPos to EndPos
        Vector2 posOnLine(double scale);
        double faceLine();
    private:
        Vector2 botReceivePos(const Vector2 &startPos, const Vector2 &botPos);
        double penaltyForBlocks(const world::WorldData &world);
        double penaltyForDistance();
        double scoreForGoalAngle(const Field &field, const world::WorldData &world);
};

}//coach
}//ai
}//rtt

#endif //ROBOTEAM_AI_POSSIBLEPASS_H
