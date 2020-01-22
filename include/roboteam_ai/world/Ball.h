#ifndef ROBOTEAM_AI_BALL_H
#define ROBOTEAM_AI_BALL_H

#include "roboteam_proto/WorldBall.pb.h"
#include "roboteam_utils/Angle.h"
#include "roboteam_utils/Vector2.h"

namespace rtt::ai::world {

class WorldData;
class Robot;

/**
 * Represents and simulates the expected state of the Ball.
 * @author Created by: Thijs Luttikhuis <br>
 *         Documented by: Haico Dorenbos
 * @since 2019-04-01
 */
class Ball {
   private:
    /* If the closest distance between the ball and our robots is smaller than this value then the ball is
     * considered to be close to that robot. */
    const float THRESHOLD_ROBOT_CLOSE_TO_BALL = 0.5;

    /* The movement friction during simulation and real life are different, because the simulation does not model
     * everything. So the movement friction has to be adjusted to compensate for this difference. */
    const float SIMULATION_FRICTION = 1.22;  // The expected movement friction of the ball during simulation.
    const float REAL_FRICTION = 0.61;        // The expected movement friction during real life performance.

    // The maximum possible factor used for the Kalman filter which is used when filtering the velocity.
    const float FILTER_MAX_FACTOR_FOR_VELOCITY = 0.8;
    /* The smallest velocity used for the Kalman filter for which we have that the factor is equal
     * THRESHOLD_ROBOT_CLOSE_TO_BALL. Larger velocities will also have THRESHOLD_ROBOT_CLOSE_TO_BALL as factor. */
    const float FILTER_VELOCITY_WITH_MAX_FACTOR = 8.0;
    /* The threshold used for the filtered velocity, if it exceeds this value then we use the velocity
    instead as estimation for the filtered velocity. More information can be found in the comment in the
    filterBallVelocity method of the Ball.cpp file. */
    const float MAXIMUM_FILTER_VELOCITY = 100.0;

   public:
    using BallPtr = std::shared_ptr<Ball>;
    using RobotPtr = std::shared_ptr<Robot>;

   private:
    Vector2 position = Vector2();        // The expected position where this ball currently is.
    const Vector2 velocity = Vector2();  // The expected current velocity of the ball.
    /* If the ball is visible by any camera. This value is true if the ball is visible by any camera and false
     * otherwise */
    bool visibleByAnyCamera = false;
    // The position where the ball is expected to end (lay still) after following his path.
    Vector2 expectedBallEndPosition = Vector2();
    // The velocity but then adjusted to determine a more realistic expectedBallEndPosition
    Vector2 filteredVelocity = Vector2();

    /**
     * When the position of the ball is not yet initialized, but was previously near to a robot then initialize
     * the position of the ball to a position close to that robot.
     */
    void initBallAtRobotPosition(const Ball &oldBall, const WorldData &worldData);

    /**
     * Adds a moving average over the Kalman filter to make the ball-velocity more stable (this method could be
     * moved to rtt_world).
     */
    void filterBallVelocity(Ball &oldBall, const WorldData &worldData);

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
    static bool exists;

    /**
     * Create an uninitialized ball (where the expected position and expected velocity are not set yet).
     */
    Ball();

    /**
     * Create a Ball object with the current data about the ball.
     * @param copy The current data about the ball
     */
    explicit Ball(const proto::WorldBall &copy);

    /**
     * Update the expected end (lay still) position of the ball after the ball has followed his path. Moreover
     * this method updates the expected position of the ball if the ball is close to a robot (which happens during
     * dribbling).
     * @param oldBall The old state of the ball
     * @param worldData The current data about the world
     */
    void updateBall(const BallPtr &oldBall, const WorldData &worldData);

    /**
     * Get the current expected position of the ball.
     * @return A vector which represents this position.
     */
    const Vector2 &getPos() const;

    /**
     * Set the new current expected position of the ball.
     * @param new_pos The new expected position of the ball.
     */
    void setPos(const Vector2 &new_pos);

    /**
     * Get the current expected velocity of the ball.
     * @return A vector which represents the direction and the magnitude of this velocity.
     */
    const Vector2 &getVel() const;
   
    /**
     * Check if the ball is visible by any camera.
     * @return True if the ball is visible by any camera, false otherwise.
     */
    bool getVisible();

    /**
     * Set the new visibility of the ball by any camera.
     * @param visible True if the ball is visible by any camera, false otherwise.
     */
    void setVisible(bool visible);

    /**
     * Get the expected position where the ball will end (lay still) after following his path.
     * @return A vector which represents this position.
     */
    const Vector2 &getExpectedBallEndPosition() const;
};

}  // namespace rtt::ai::world

#endif  // ROBOTEAM_AI_BALL_H
