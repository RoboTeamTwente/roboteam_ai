//
// Created by thijs on 19-3-19.
//

#ifndef ROBOTEAM_AI_PROCESSEDWORLDEE_H
#define ROBOTEAM_AI_PROCESSEDWORLDEE_H

#include "WorldData.h"
#include "LastWorld.h"

namespace rtt {
namespace ai {
namespace world {

class ProcessedWorld {
    private:
        using RobotPtr = std::shared_ptr<Robot>;
        using BallPtr = std::shared_ptr<Ball>;
        using WorldDataPtr = std::shared_ptr<WorldData>;

        void updateFutureRobot(Robot &robot, double time);
        void updateFutureBall(Ball &ball, double time);

        void updateBallPosition(WorldData &worldData);
        void updateBallPossession(WorldData &worldData);
    public:
        void update(WorldData worldData);
        RobotPtr getRobotClosestToPoint(const Vector2 &point, std::vector<Robot> &robots);
        void updateFutureWorld(WorldData &worldData, double time);
};

extern ProcessedWorld processedWorldObj;
extern ProcessedWorld* processedWorld;

}
}
}

#endif //ROBOTEAM_AI_PROCESSEDWORLD_H
