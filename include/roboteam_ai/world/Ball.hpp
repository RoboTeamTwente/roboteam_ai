//
// Created by john on 12/18/19.
//

#ifndef RTT_BALL_HPP
#define RTT_BALL_HPP

#include <proto/WorldBall.pb.h>

#include "roboteam_utils/Vector2.h"

namespace rtt::world {
class World;
}

namespace rtt::world::ball {

class Ball {
public:
    Vector2 position; /// Position of the ball
    Vector2 velocity; /// Velocity of the ball
    bool visible = false; /// Whether the ball is visible by any camera
    /// Expected position of the ball after it stopped moving

    /**
     * Initializes ball at the previously seen position, if the current ball is not visible
     */
    void initBallAtExpectedPosition(const world::World *data) noexcept;

    /**
     * Updates the expected ball end position
     */
    void updateExpectedBallEndPosition(const world::World *data) noexcept;

    /**
     * Places the ball in front of the robot that has the ball, if any
     */
    void updateBallAtRobotPosition(const world::World *data) noexcept;

    /**
     * Create a Ball object with the current data about the ball.
     * @param copy The current data about the ball
     */
    explicit Ball(const proto::WorldBall &copy, const world::World *data);

    /**
     * Defaulted constructors
     */
    Ball() = default;
    Ball &operator=(Ball const &) = default;
    Ball(Ball const &) = default;
    Ball &operator=(Ball &&) = default;
    Ball(Ball &&) = default;
    ~Ball() = default;
};

}  // namespace rtt::world::ball

#endif  // RTT_BALL_HPP
