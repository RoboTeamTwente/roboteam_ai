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
    public:
        using BallPtr = std::shared_ptr<Ball>;
        using RobotPtr = std::shared_ptr<Robot>;

        const BallPtr deepCopy() const;

    private:
        bool ballInAir = false;
        bool collidesNow = false;
        bool kickedNow = false;
        bool dribbledNow = false;
        double lastKickVel = 0;
        int ballStraightTicks = 0;

        void updateDribbling(const Ball &oldBall, const WorldData &worldData);
        Robot* getDribblingRobot(const std::vector<RobotPtr> &robots, double maxDribbleRange);
        void updateBallModel(const Ball &oldBall, const WorldData &worldData);
        void updateBallPosition(const Ball &oldBall, const WorldData &worldData);

    public:
        Ball();
        // Ball(const Ball &copy) = default;
        explicit Ball(const roboteam_msgs::WorldBall &copy);
        void updateBall(const BallPtr &oldBall, const WorldData &worldData);

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
