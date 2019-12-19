//
// Created by john on 12/18/19.
//

#include "roboteam_world/world/ball.hpp"

namespace rtt::world::ball {

    Ball::Ball(const proto::WorldBall &copy)
        : position { copy.pos() }, velocity{ copy.vel() },
          filteredVelocity{ velocity }, visible{ copy.visible() } {
    }

    void Ball::initBallAtRobotPosition(const rtt::world::ball::Ball &oldBall,
                                       const world::WorldData &worldData) {

    }


    /**
     initBallAtRobotPosition(oldBall, worldData);
     filterBallVelocity(oldBall, worldData);
     updateExpectedBallEndPosition(oldBall, worldData);
     updateBallAtRobotPosition(oldBall, worldData);
    **/

} // namespace rtt::world::ball
