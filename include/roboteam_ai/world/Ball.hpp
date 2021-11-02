//
// Created by john on 12/18/19.
//

#ifndef RTT_BALL_HPP
#define RTT_BALL_HPP

#include <roboteam_proto/WorldBall.pb.h>
#include "roboteam_utils/Vector2.h"

namespace rtt::world {
    class World;
}

namespace rtt::world::ball {

/**
 * The movement friction during simulation and real life are different, because the simulation does not model
 * everything. So the movement friction has to be adjusted to compensate for this difference.
 *
 * The expected movement friction of the ball during simulation
 */
constexpr static float SIMULATION_FRICTION = 1.22;

/**
 * The expected movement friction of the ball during simulation
 */
constexpr static float REAL_FRICTION = 0.61;

/**
 * The maximum possible factor used for the Kalman filter which is used when filtering the velocity.
 */
constexpr static float FILTER_MAX_FACTOR_FOR_VELOCITY = 0.8;

/**
 * The smallest velocity used for the Kalman filter for which we have that the factor is equal
 * THRESHOLD_ROBOT_CLOSE_TO_BALL. Larger velocities will also have THRESHOLD_ROBOT_CLOSE_TO_BALL as factor.
 */
constexpr static float FILTER_VELOCITY_WITH_MAX_FACTOR = 8.0;

/**
 * The threshold used for the filtered velocity, if it exceeds this value then we use the velocity
 * instead as estimation for the filtered velocity. More information can be found in the comment in the
 * filterBallVelocity method of the Ball.cpp file.
 */
constexpr static float MAXIMUM_FILTER_VELOCITY = 100.0;

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
    void initializeCalculations(const world::World* data) noexcept;

    /**
     * Initializes ball at the robot's position if `this` does not have a position
     */
    void initBallAtRobotPosition(const world::World* data) noexcept;

    /**
     * Sets filteredVelocity
     */
    void filterBallVelocity(const world::World* data) noexcept;

    /**
     * Updates the expected ball end position
     */
    void updateExpectedBallEndPosition(const world::World* data) noexcept;

    /**
     * Updates expectedEndPosition and draws to interface
     * If ball not visible -> get last position if robot can confirm
     * Also updates which robot has the ball and the location
     */
    void updateBallAtRobotPosition(const world::World* data) noexcept;

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
    explicit Ball(const proto::WorldBall &copy, const world::World* data);

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
