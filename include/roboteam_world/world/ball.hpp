//
// Created by john on 12/18/19.
//

#ifndef RTT_BALL_HPP
#define RTT_BALL_HPP

#include "../roboteam_utils/include/roboteam_utils/Vector2.h"

#include "../roboteam_world/include/roboteam_world/world/world_data.hpp"
#include "world_data.hpp"

namespace rtt::world::ball {
    constexpr static float THRESHOLD_ROBOT_CLOSE_TO_BALL = 0.5;
    constexpr static float SIMULATION_FRICTION = 1.22;
    constexpr static float REAL_FRICTION = 0.61;
    constexpr static float FILTER_MAX_FACTOR_FOR_VELOCITY = 0.8;
    constexpr static float FILTER_VELOCITY_WITH_MAX_FACTOR = 8.0;
    constexpr static float MAXIMUM_FILTER_VELOCITY = 100.0;
    
    class Ball {
        Vector2 position;
    public:
        const Vector2 &getPosition() const;

        const Vector2 &getVelocity() const;

        bool isVisible() const;

        const Vector2 &getExpectedEndPosition() const;

        const Vector2 &getFilteredVelocity() const;

    private:
        Vector2 velocity;
        bool visible = false;
        Vector2 expectedEndPosition;
        Vector2 filteredVelocity;
    private:
        /**
         * When the position of the ball is not yet initialized, but was previously near to a robot then initialize
         * the position of the ball to a position close to that robot.
         */
        void initBallAtRobotPosition(const Ball &oldBall, const WorldData &worldData);

        /**
         * Adds a moving average over the Kalman filter to make the ball-velocity more stable (this method could be
         * moved to rtt_world).
        */
        void filterBallVelocity(Ball const& oldBall, const WorldData &worldData);

        /**
         * Update the expected position where the ball will end (lay still) after following his path.
         */
        void updateExpectedBallEndPosition(const Ball &oldBall, const WorldData &worldData);

        /**
         * When the position of the ball is close to a robot and not visible by camera then update the ball according
         * to the movement and rotation of that robot (which happens during dribbling).
         */
        void updateBallAtRobotPosition(const Ball &oldBall, const WorldData &worldData);

    public:
        Ball() = default;

        /**
         * Create a Ball object with the current data about the ball.
         * @param copy The current data about the ball
         */
        explicit Ball(const proto::WorldBall &copy);



    };

} // namespace rtt::world::ball


#endif //RTT_BALL_HPP
