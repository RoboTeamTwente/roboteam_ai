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
class DefensiveCoach {
    private:
        //BLOCKPOS actively covers the goal up from the ball or robots who can receive the ball and score
        //PASSIVEINTERCEPTPOS tries to go to a position where in case of a pass to another robot it can intercept it in case of a pass
        //ACTIVEINTERCEPTPOS is actively blocking passes  to other robots (e.g. on the path)

        std::map<int, std::pair<Vector2, double>> defenderLocations;
        std::vector<int> defenders;
        //pass detection
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

        Vector2 getPos(std::pair<Vector2, Vector2> line, double aggressionFactor);
        void visualizePoints();

        std::vector<roboteam_msgs::WorldRobot> createVirtualBots(std::vector<Vector2> decidedBlocks);
        std::vector<std::pair<PossiblePass, double>> createPassesAndDanger(std::vector<roboteam_msgs::WorldRobot> bots,
                std::vector<Vector2> decidedBlocks);
    public:
        std::shared_ptr<std::pair<Vector2, Vector2>> getBlockLineSegment(std::pair<Vector2, Vector2> openGoalSegment,
                Vector2 point, double collisionRadius, double margin = - 1.0);
        std::vector<Vector2> doubleBlockOnDefenseLine(std::pair<Vector2, Vector2> openGoalSegment, Vector2 point);
        std::shared_ptr<Vector2> blockOnDefenseLine(std::pair<Vector2, Vector2> openGoalSegment, Vector2 point);
        std::vector<std::pair<Vector2, double>> decideDefendersOnDefenseLine(int amount);

        void updateDefenderLocations();
        void addDefender(int id);
        void removeDefender(int id);
        std::shared_ptr<std::pair<Vector2, double>> getDefenderPosition(int id);

        Vector2 getBlockPoint(std::pair<Vector2, Vector2> openGoalSegment, Vector2 point, double collisionRadius);
        std::pair<Vector2, Vector2> shortenLineForDefenseArea(Vector2 lineStart, Vector2 lineEnd, double defenseMargin);
        Vector2 computeSimpleReceivePos(Vector2 startPos, Vector2 robotPos);
};
extern DefensiveCoach g_defensiveCoach;

}
}
}

#endif //ROBOTEAM_AI_DEFENSIVECOACH_H
