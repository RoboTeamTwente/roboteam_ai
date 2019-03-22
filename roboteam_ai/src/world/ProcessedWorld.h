//
// Created by thijs on 19-3-19.
//

#ifndef ROBOTEAM_AI_PROCESSEDWORLD_H
#define ROBOTEAM_AI_PROCESSEDWORLD_H

#include "WorldData.h"

namespace rtt {
namespace ai {
namespace world {

class World;
class ProcessedWorld {
    private:
        using RobotPtr = std::shared_ptr<Robot>;
        using BallPtr = std::shared_ptr<Ball>;
        using WorldDataPtr = std::shared_ptr<WorldData>;

        void updateFutureRobot(Robot &robot, double time);
        void updateFutureBall(Ball &ball, double time);
    public:
        void update();
        RobotPtr getRobotClosestToPoint(const Vector2 &point, std::vector<Robot> &robots);
        void updateFutureWorld(WorldData &worldData, double time);
};

}
}
}

#endif //ROBOTEAM_AI_PROCESSEDWORLD_H
