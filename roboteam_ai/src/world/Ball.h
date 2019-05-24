//
// Created by thijs on 1-4-19.
//

#ifndef ROBOTEAM_AI_BALL_H
#define ROBOTEAM_AI_BALL_H

#include "roboteam_msgs/WorldBall.h"

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Angle.h"

namespace rtt {
namespace ai {
namespace world {
class WorldData;
class Robot;

class Ball {
    private:
        using BallPtr = std::shared_ptr<Ball>;
        using RobotPtr = std::shared_ptr<Robot>;

        bool ballInAir;
        bool collidesNow;
        bool kickedNow;
        bool dribbledNow;
        double lastKickVel;
        int ballStraightTicks;

        void updateDribbling(const Ball &oldBall, const WorldData &worldData);
        Robot* getDribblingRobot(const std::vector<RobotPtr> &robots, double maxDribbleRange);
        void updateBallModel(const Ball &oldBall, const WorldData &worldData);
        void updateBallPosition(const Ball &oldBall, const WorldData &worldData);

    public:
        Ball();
        explicit Ball(const roboteam_msgs::WorldBall &copy);
        void updateBall(const Ball &oldBall, const WorldData &worldData);

        Vector2 pos = Vector2();
        Vector2 vel = Vector2();
        Vector2 acc = Vector2();
        double spin = 0.0;
        static bool exists;
        bool visible = false;
};

}
}
}

#endif //ROBOTEAM_AI_BALL_H
