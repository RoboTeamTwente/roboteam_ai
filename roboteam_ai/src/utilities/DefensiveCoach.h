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

        enum PosType{BLOCKPOS, PASSIVEINTERCEPTPOS};
        std::map<int,Vector2> robotWithPos;
        //pass detection
        struct PossiblePass{
          roboteam_msgs::WorldRobot toBot,fromBot;
          Vector2 startPos;
          Vector2 endPos;

          double distance();
          bool obstacleObstructsPath(Vector2 obstPos, double obstRadius=(Constants::ROBOT_RADIUS()+Constants::BALL_RADIUS()));
          int amountOfBlockers();
          PossiblePass(roboteam_msgs::WorldRobot _toBot,roboteam_msgs::WorldRobot _fromBot, Vector2 ballPos );
        };
    public:
        static std::vector<PossiblePass> getPossiblePassesThem();
        static std::shared_ptr<std::pair<Vector2,Vector2>> getBlockLineSegment(std::pair<Vector2,Vector2> openGoalSegment, Vector2 point,double collisionRadius);
        static std::shared_ptr<std::pair<Vector2,Vector2>> blockBall();
        static std::vector<std::pair<Vector2,Vector2>> getWholeBlockSegments(std::vector<Vector2>);
};

}
}
}


#endif //ROBOTEAM_AI_DEFENSIVECOACH_H
