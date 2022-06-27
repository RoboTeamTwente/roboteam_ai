//
// Created by john on 12/18/19.
//

#include "world/Ball.hpp"

#include "interface/api/Input.h"
#include "utilities/Constants.h"
#include "world/World.hpp"

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

Ball::Ball(const proto::WorldBall& copy, const World* data) : position{copy.pos().x(), copy.pos().y()}, velocity{copy.vel().x(), copy.vel().y()}, visible{copy.visible()} {
    initializeCalculations(data);
}

const Vector2& Ball::getPos() const noexcept { return position; }

const Vector2& Ball::getVelocity() const noexcept { return velocity; }

bool Ball::isVisible() const noexcept { return visible; }

const Vector2& Ball::getExpectedEndPosition() const noexcept { return expectedEndPosition; }

void Ball::initializeCalculations(const world::World* data) noexcept {
    initBallAtRobotPosition(data);
    updateExpectedBallEndPosition(data);
    updateBallAtRobotPosition(data);
}

void Ball::initBallAtRobotPosition(const world::World* data) noexcept {
    std::optional<view::WorldDataView> previousWorld = data->getHistoryWorld(1);

    if (!previousWorld) {
        return;
    }

    auto optionalPreviousBall = previousWorld->getBall();

    if (!optionalPreviousBall.has_value()) {
        return;
    }

    auto previousBall = optionalPreviousBall.value();

    if (position != Vector2() || previousBall->getPos() == Vector2()) {
        return;
    }

    // Current ball does not have a position, set it to the old pos
    auto rbtView = previousWorld->getRobotClosestToBall(us);
    if (rbtView) {
        this->position = previousBall->getPos();
    }
}

void Ball::updateExpectedBallEndPosition(const world::World* data) noexcept {
    std::optional<view::WorldDataView> previousWorld = data->getHistoryWorld(1);

    if (!previousWorld) {
        return;
    }

    auto optionalPreviousBall = previousWorld->getBall();

    if (!optionalPreviousBall) {
        return;
    }

    auto ball = optionalPreviousBall.value();

    double ballVelSquared = ball->getVelocity().length2();
    const double frictionCoefficient = ai::Constants::GRSIM() ? SIMULATION_FRICTION : REAL_FRICTION;

    expectedEndPosition = ball->getPos() + ball->velocity.stretchToLength(ballVelSquared / frictionCoefficient);

    // Visualize the Expected Ball End Position
    ai::interface::Input::drawData(ai::interface::Visual::BALL_DATA, {getExpectedEndPosition()}, ai::Constants::BALL_COLOR(), -1, ai::interface::Drawing::CIRCLES, 8, 8, 6);
    ai::interface::Input::drawData(ai::interface::Visual::BALL_DATA, {position, getExpectedEndPosition()}, ai::Constants::BALL_COLOR(), -1,
                                   ai::interface::Drawing::LINES_CONNECTED);
}

void Ball::updateBallAtRobotPosition(const world::World* data) noexcept {
    if (isVisible()) {
        return;
    }

    std::optional<view::WorldDataView> world = data->getWorld();

    if (!world.has_value()) return;

    std::optional<rtt::world::view::RobotView> robotWithBall = world->whichRobotHasBall();
    if (!robotWithBall.has_value()) {
        return;
    }

    std::pair<uint8_t, Team> newRobotIdTeam = {(*robotWithBall)->getId(), (*robotWithBall)->getTeam()};
    view::RobotView newRobotWithBall{nullptr};
    for (auto robot : world->getUs()) {
        if (robot->getId() != newRobotIdTeam.first || robot->getTeam() != newRobotIdTeam.second) {
            continue;
        }
        newRobotWithBall = robot;
    }
    if (!newRobotWithBall) {
        return;
    }
    double distanceInFrontOfRobot = ai::Constants::ROBOT_RADIUS() + ai::Constants::BALL_RADIUS();
    position = newRobotWithBall->getPos() + newRobotWithBall->getAngle().toVector2(distanceInFrontOfRobot);
}

}  // namespace rtt::world::ball
