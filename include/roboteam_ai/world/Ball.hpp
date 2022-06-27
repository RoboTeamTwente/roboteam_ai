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
   private:
    /**
     * The position where this ball currently is
     */
    Vector2 position;

    /**
     * The current velocity of the ball
     */
    Vector2 velocity;

    /**
     * Boolean flag that indicates whether the ball is currently visible by any camera
     */
    bool visible = false;

    /**
     * Expected ball end position (where it lays still) after following its path
     */
    Vector2 expectedEndPosition;

    /**
     * The velocity but thn adjusted to determine a more realistic end position
     */
    Vector2 filteredVelocity;

    /**
     * Initializes:
     *  Filtered velocity
     *  Expected end pos
     *  Sets position if it's currently unknown
     *  Updates position
     */
    void initializeCalculations(const world::World *data) noexcept;

    /**
     * Initializes ball at the robot's position if `this` does not have a position
     */
    void initBallAtRobotPosition(const world::World *data) noexcept;

    /**
     * Sets filteredVelocity
     */
    void filterBallVelocity(const world::World *data) noexcept;

    /**
     * Updates the expected ball end position
     */
    void updateExpectedBallEndPosition(const world::World *data) noexcept;

    /**
     * Updates expectedEndPosition and draws to interface
     * If ball not visible -> get last position if robot can confirm
     * Also updates which robot has the ball and the location
     */
    void updateBallAtRobotPosition(const world::World *data) noexcept;

   public:
    [[nodiscard]] const Vector2 &getPos() const noexcept;

    [[nodiscard]] const Vector2 &getVelocity() const noexcept;

    [[nodiscard]] bool isVisible() const noexcept;

    [[nodiscard]] const Vector2 &getExpectedEndPosition() const noexcept;

    [[nodiscard]] const Vector2 &getFilteredVelocity() const noexcept;

    /**
     * Default ctor for containers
     */
    Ball() = default;

    /**
     * Create a Ball object with the current data about the ball.
     * @param copy The current data about the ball
     */
    explicit Ball(const proto::WorldBall &copy, const world::World *data);

    /**
     * Defaulted constructors
     */
    Ball &operator=(Ball const &) = default;

    Ball(Ball const &) = default;

    Ball &operator=(Ball &&) = default;

    Ball(Ball &&) = default;

    ~Ball() = default;
};

}  // namespace rtt::world::ball

#endif  // RTT_BALL_HPP
