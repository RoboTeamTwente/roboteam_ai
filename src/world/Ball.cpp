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
 * The expected movement friction of the ball on the field
 */
constexpr static float REAL_FRICTION = 0.5;


Ball::Ball(const proto::WorldBall& copy, const World* data) : position{copy.pos().x(), copy.pos().y()}, velocity{copy.vel().x(), copy.vel().y()}, visible{copy.visible()} {
    if (!visible || position == Vector2()){
        initBallAtExpectedPosition(data);
        updateBallAtRobotPosition(data);
    }
    updateExpectedBallEndPosition(data);
}

void Ball::initBallAtExpectedPosition(const world::World* data) noexcept {
    std::optional<view::WorldDataView> previousWorld = data->getHistoryWorld(1);

    if (!previousWorld || !previousWorld->getBall()) {
        return;
    }
    position = previousWorld->getBall().value()->position;
}

void Ball::updateExpectedBallEndPosition(const world::World* data) noexcept {
    std::optional<view::WorldDataView> previousWorld = data->getHistoryWorld(1);

    if (!previousWorld || !previousWorld->getBall()) {
        return;
    }

    auto ball = previousWorld->getBall().value();

    double ballVelSquared = ball->velocity.length2();
    const double frictionCoefficient = SETTINGS.getRobotHubMode() == Settings::RobotHubMode::SIMULATOR ? SIMULATION_FRICTION : REAL_FRICTION;

    ball->position + ball->velocity.stretchToLength(ballVelSquared / frictionCoefficient);

    // Visualize the Expected Ball End Position
    //ai::interface::Input::drawData(ai::interface::Visual::BALL_DATA, {expectedEndPosition}, ai::Constants::BALL_COLOR(), -1, ai::interface::Drawing::CIRCLES, 8, 8, 6);
    //ai::interface::Input::drawData(ai::interface::Visual::BALL_DATA, {position, expectedEndPosition}, ai::Constants::BALL_COLOR(), -1,
    //                               ai::interface::Drawing::LINES_CONNECTED);
}

void Ball::updateBallAtRobotPosition(const world::World* data) noexcept {
    std::optional<view::WorldDataView> world = data->getWorld();
    if (!world.has_value()) return;

    std::optional<rtt::world::view::RobotView> robotWithBall = world->whichRobotHasBall();
    if (!robotWithBall.has_value()) {
        return;
    }
    // Place the ball where we would expect it to be given that this robot has the ball
    double distanceInFrontOfRobot = ai::stp::control_constants::CENTER_TO_FRONT + ai::Constants::BALL_RADIUS();
    position = robotWithBall->get()->getPos() + robotWithBall->get()->getAngle().toVector2(distanceInFrontOfRobot);
}

}  // namespace rtt::world::ball
