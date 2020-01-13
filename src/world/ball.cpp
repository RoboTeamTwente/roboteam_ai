//
// Created by john on 12/18/19.
//

#include "world/ball.hpp"
#include "world/world.hpp"

namespace rtt::world_new::ball {

    Ball::Ball(const proto::WorldBall &copy)
            : position { copy.pos() }, velocity{ copy.vel() },
              filteredVelocity{ velocity }, visible{ copy.visible() } {
    }

    const Vector2 &Ball::getPos() const noexcept {
        return position;
    }

    const Vector2 &Ball::getVelocity() const noexcept{
        return velocity;
    }

    bool Ball::isVisible() const noexcept{
        return visible;
    }

    const Vector2 &Ball::getExpectedEndPosition() const noexcept{
        return expectedEndPosition;
    }

    const Vector2 &Ball::getFilteredVelocity() const noexcept{
        return filteredVelocity;
    }


    /**
     initBallAtRobotPosition(oldBall, worldData);
     filterBallVelocity(oldBall, worldData);
     updateExpectedBallEndPosition(oldBall, worldData);
     updateBallAtRobotPosition(oldBall, worldData);
    **/

} // namespace rtt::world_new::ball
