//
// Created by john on 12/18/19.
//

#ifndef RTT_BALL_HPP
#define RTT_BALL_HPP

#include "roboteam_utils/Vector2.h"
#include "roboteam_proto/WorldBall.pb.h"

namespace rtt::world_new::ball {

    constexpr static float THRESHOLD_ROBOT_CLOSE_TO_BALL = 0.5;
    constexpr static float SIMULATION_FRICTION = 1.22;
    constexpr static float REAL_FRICTION = 0.61;
    constexpr static float FILTER_MAX_FACTOR_FOR_VELOCITY = 0.8;
    constexpr static float FILTER_VELOCITY_WITH_MAX_FACTOR = 8.0;
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
        void initializeCalculations() noexcept;

        /**
         * Initializes ball at the robot's position if `this` does not have a position
         */
        void initBallAtRobotPosition() noexcept;

        /**
         * Sets filteredVelocity
         */
        void filterBallVelocity() noexcept;

        /**
         * Updates the expected ball end position
         */
        void updateExpectedBallEndPosition() noexcept;

        /**
         * Draws expectedEndPosition and draws to inteface
         */
        void updateBallAtRobotPosition() noexcept;


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
        explicit Ball(const proto::WorldBall &copy);

        /**
         * Defaulted constructors
         */
        Ball &operator=(Ball const &) = default;

        Ball(Ball const &) = default;

        Ball &operator=(Ball &&) = default;

        Ball(Ball &&) = default;

        ~Ball() = default;

    };

} // namespace rtt::world::ball


#endif //RTT_BALL_HPP
