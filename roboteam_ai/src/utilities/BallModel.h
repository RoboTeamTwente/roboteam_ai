//
// Created by rolf on 13-3-19.
//

#ifndef ROBOTEAM_AI_BALLMODEL_H
#define ROBOTEAM_AI_BALLMODEL_H
#include "World.h"
namespace rtt {
namespace ai {
class BallModel {
    private:
        static roboteam_msgs::WorldBall currentBall;
        static bool ballInAir;
        static bool collidesNow;
        static bool kickedNow;
        static double lastKickVel;
        static int ballStraightTicks;

    public:
        static void updateBallModel(roboteam_msgs::WorldBall newBall);
        static bool isBallInAir();
        static bool ballCollided();
        static bool ballKicked();
        struct simulatedBall{
          Vector2 pos;
          Vector2 vel;
          int collissions=0;
        };
        static std::vector<simulatedBall> extrapolateBallSimple(double timeAhead, double timeStep);

};
}
}

#endif //ROBOTEAM_AI_BALLMODEL_H
