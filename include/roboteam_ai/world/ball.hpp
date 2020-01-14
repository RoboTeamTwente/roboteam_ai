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
        Vector2 position;
        Vector2 velocity;
        bool visible = false;
        Vector2 expectedEndPosition;
        Vector2 filteredVelocity;

    public:
//        /**
//         * When the position of the ball is not yet initialized, but was previously near to a robot then initialize
//         * the position of the ball to a position close to that robot.
//         */
//        void initBallAtRobotPosition(const Ball &oldBall, const WorldData &worldData);
//
//        /**
//         * Adds a moving average over the Kalman filter to make the ball-velocity more stable (this method could be
//         * moved to rtt_world).
//        */
//        void filterBallVelocity(Ball const& oldBall, const WorldData &worldData);
//
//        /**
//         * Update the expected position where the ball will end (lay still) after following his path.
//         */
//        void updateExpectedBallEndPosition(const Ball &oldBall, const WorldData &worldData);
//
//        /**
//         * When the position of the ball is close to a robot and not visible by camera then update the ball according
//         * to the movement and rotation of that robot (which happens during dribbling).
//         */
//        void updateBallAtRobotPosition(const Ball &oldBall, const WorldData &worldData);


        [[nodiscard]] const Vector2 &getPos() const noexcept;

        [[nodiscard]] const Vector2 &getVelocity() const noexcept;

        [[nodiscard]] bool isVisible() const noexcept;

        [[nodiscard]] const Vector2 &getExpectedEndPosition() const noexcept;

        [[nodiscard]] const Vector2 &getFilteredVelocity() const noexcept;

        Ball() = default;

        /**
         * Create a Ball object with the current data about the ball.
         * @param copy The current data about the ball
         */
        explicit Ball(const proto::WorldBall &copy);

        Ball& operator=(Ball const&) = default;
        Ball(Ball const&) = default;

        Ball& operator=(Ball&&) = default;
        Ball(Ball&&) = default;

    };

} // namespace rtt::world::ball


#endif //RTT_BALL_HPP
