//
// Created by rolf on 18-2-19.
//

#ifndef ROBOTEAM_AI_DEFENSIVECOACH_H
#define ROBOTEAM_AI_DEFENSIVECOACH_H
#include "roboteam_utils/Vector2.h"
#include "roboteam_ai/src/utilities/World.h"
namespace rtt {
namespace ai {
namespace coach {
class DefencePositionCoach {
    private:
        Vector2 getPos(std::pair<Vector2, Vector2> line, double aggressionFactor);

        std::vector<roboteam_msgs::WorldRobot> createVirtualBots(std::vector<Vector2> decidedBlocks);

    public:
        struct PossiblePass {
          roboteam_msgs::WorldRobot toBot;
          Vector2 startPos;
          Vector2 endPos;
          double distance();
          bool obstacleObstructsPath(Vector2 obstPos,
                  double obstRadius = (Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS()));
          int amountOfBlockers();
          PossiblePass(roboteam_msgs::WorldRobot _toBot, Vector2 ballPos);
        };
        double scorePossiblePass(PossiblePass pass, std::vector<Vector2> decidedBlocks);
        double scoreForOpenGoalAngle(PossiblePass pass, std::vector<Vector2> decidedBlocks);
        double penaltyForBlocks(PossiblePass pass, std::vector<Vector2> decidedBlocks);
        double penaltyForPassDist(PossiblePass pass);
        Vector2 computeSimpleReceivePos(Vector2 startPos, Vector2 robotPos);

        enum botType{BLOCKBALLTOGOAL,BLOCKTOGOAL,WALL,BLOCKPASS,BLOCKONLINE};
        struct DefenderBot{
          int id;
          Vector2 targetPos;
          double orientation;
          int blockFromID;
          botType type;
        };
        struct BlockPassBot:DefenderBot{
          int blockToID;
        };

        std::vector<Vector2> doubleBlockOnDefenseLine(std::pair<Vector2, Vector2> openGoalSegment, Vector2 point);
        std::vector<std::pair<Vector2, double>> decideDefendersOnDefenseLine(int amount);

        std::shared_ptr<DefenderBot> createBlockToGoal(PossiblePass pass, double aggressionFactor);
        std::shared_ptr<BlockPassBot> createBlockPass(PossiblePass pass,double closeToPasserFactor);
        std::shared_ptr<DefenderBot> createBlockOnLine(PossiblePass pass);

        std::shared_ptr<std::pair<Vector2, Vector2>> getBlockLineSegment(std::pair<Vector2, Vector2> openGoalSegment,
                Vector2 point, double collisionRadius, double margin = - 1.0);
        std::shared_ptr<Vector2> blockOnDefenseLine(std::pair<Vector2, Vector2> openGoalSegment, Vector2 point);
        Vector2 getBlockPoint(std::pair<Vector2, Vector2> openGoalSegment, Vector2 point, double collisionRadius);
        std::pair<Vector2, Vector2> shortenLineForDefenseArea(Vector2 lineStart, Vector2 lineEnd, double defenseMargin);
    private:
        std::vector<std::pair<PossiblePass, double>> createPassesAndDanger(std::vector<roboteam_msgs::WorldRobot> bots,
                std::vector<Vector2> decidedBlocks);
};
extern DefencePositionCoach g_defensiveCoach;

}
}
}

#endif //ROBOTEAM_AI_DEFENSIVECOACH_H
