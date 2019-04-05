//
// Created by thijs on 19-3-19.
//

#ifndef ROBOTEAM_AI_FUTUREWORLD_H
#define ROBOTEAM_AI_FUTUREWORLD_H

#include "WorldData.h"

namespace rtt {
namespace ai {
namespace world {

class FutureWorld {
    private:
        using RobotPtr = std::shared_ptr<Robot>;
        using BallPtr = std::shared_ptr<Ball>;
        using WorldDataPtr = std::shared_ptr<WorldData>;

    public:
        void updateFutureRobot(Robot &robot, double time);
        void updateFutureBall(Ball &ball, double time);
        void updateFutureWorld(WorldData &worldData, double time);
};

}
}
}

#endif //ROBOTEAM_AI_PROCESSEDWORLD_H
