//
// Created by rolf on 5-4-19.
//

#ifndef ROBOTEAM_AI_POSSIBLEPASS_H
#define ROBOTEAM_AI_POSSIBLEPASS_H
#include <roboteam_utils/Vector2.h>
#include <roboteam_msgs/World.h>
#include <roboteam_ai/src/utilities/Constants.h>
namespace rtt{
namespace ai{
namespace coach{
using Line=std::pair<Vector2, Vector2>;
class PossiblePass {
    public:
        world::Robot *toBot;
        Vector2 startPos;
        Vector2 endPos;
        double distance();
        bool obstacleObstructsPath(const Vector2& obstPos,
                double obstRadius = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
        int amountOfBlockers(const world::WorldData& world);
        PossiblePass(world::Robot *_toBot, const Vector2& ballPos);
        double score(const world::WorldData& world);
    private:
        Vector2 botReceivePos(const Vector2& startPos, const Vector2& botPos);
        double penaltyForBlocks(const world::WorldData& world);
        double penaltyForDistance();
        double scoreForGoalAngle(const world::WorldData& world);
};
}
}
}


#endif //ROBOTEAM_AI_POSSIBLEPASS_H
