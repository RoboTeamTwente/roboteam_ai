//
// Created by thijs on 19-3-19.
//

#ifndef ROBOTEAM_AI_FUTUREWORLD_H
#define ROBOTEAM_AI_FUTUREWORLD_H

#include "WorldData.h"

namespace rtt::ai::world {

    class FutureWorld {
        private:
        using RobotPtr = std::shared_ptr<Robot>;
        using BallPtr = std::shared_ptr<Ball>;
        using WorldDataPtr = std::shared_ptr<WorldData>;

        public:
        void updateFutureRobot(RobotPtr &robot, double time);
        void updateFutureBall(BallPtr &ball, double time);
        void updateFutureWorld(WorldData &worldData, double time);
    };

}  // namespace rtt::ai::world

#endif  // ROBOTEAM_AI_PROCESSEDWORLD_H
