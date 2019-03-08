//
// Created by rolf on 18-2-19.
//

#ifndef ROBOTEAM_AI_DEFENSIVECOACH_H
#define ROBOTEAM_AI_DEFENSIVECOACH_H
#include "roboteam_utils/Vector2.h"
#include "../utilities/World.h"
namespace rtt{
namespace ai{
namespace coach{
class DefensiveCoach {
    private:
        //BLOCKPOS actively covers the goal up from the ball or robots who can receive the ball and score
        //PASSIVEINTERCEPTPOS tries to go to a position where in case of a pass to another robot it can intercept it in case of a pass
        //ACTIVEINTERCEPTPOS is actively blocking passes  to other robots (e.g. on the path)

        static std::map<int,std::pair<Vector2,double>> defenderLocations;
        static std::vector<int> defenders;
        //pass detection
        struct PossiblePass{
          roboteam_msgs::WorldRobot toBot;
          Vector2 startPos;
          Vector2 endPos;

          double distance();
          bool obstacleObstructsPath(Vector2 obstPos, double obstRadius=(Constants::ROBOT_RADIUS()+Constants::BALL_RADIUS()));
          int amountOfBlockers();
          PossiblePass(roboteam_msgs::WorldRobot _toBot,Vector2 ballPos );
        };
        static double scorePossiblePass(PossiblePass pass);
        static double scorePossiblePass2(PossiblePass pass, std::vector<Vector2> decidedBlocks);
        static double scorePossiblePass3(PossiblePass pass, std::vector<Vector2> decidedBlocks);
        static Vector2 getPos(std::pair<Vector2,Vector2> line, double aggressionFactor);
    public:
        static std::vector<PossiblePass> getPossiblePassesThem();
        static std::shared_ptr<std::pair<Vector2,Vector2>> getBlockLineSegment(std::pair<Vector2,Vector2> openGoalSegment, Vector2 point,double collisionRadius);
        static std::shared_ptr<std::pair<Vector2,Vector2>> blockBall();
        static std::vector<std::pair<Vector2,Vector2>> decideDefenderLocations(int amount);
        static std::vector<Vector2> decideDefenderLocations2(int amount,double aggressionFactor);
        static std::vector<Vector2> doubleBlockOnDefenseLine(std::pair<Vector2, Vector2> openGoalSegment, Vector2 point);
        static std::shared_ptr<Vector2> blockOnDefenseLine(std::pair<Vector2, Vector2> openGoalSegment, Vector2 point);
        static std::vector<std::pair<Vector2,double>> decideDefendersOnDefenseLine(int amount);

        static void updateDefenderLocations();
        static void addDefender(int id);
        static void removeDefender(int id);
        static std::shared_ptr<std::pair<Vector2,double>> getDefenderPosition(int id);

};

}
}
}


#endif //ROBOTEAM_AI_DEFENSIVECOACH_H
