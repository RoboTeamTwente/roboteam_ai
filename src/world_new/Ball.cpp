//
// Created by john on 12/18/19.
//

#include <include/roboteam_ai/utilities/Constants.h>
#include <include/roboteam_ai/interface/api/Input.h>
#include "world_new/Ball.hpp"
#include "world_new/World.hpp"

namespace rtt::world_new::ball {

    Ball::Ball(const proto::WorldBall &copy)
            : position{copy.pos()}, velocity{copy.vel()},
              visible{copy.visible()} {
        initializeCalculations();
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

    const Vector2 &Ball::getExpectedEndPosition() const noexcept {
        return expectedEndPosition;
    }

    const Vector2 &Ball::getFilteredVelocity() const noexcept {
        return filteredVelocity;
    }

    void Ball::initializeCalculations() noexcept {
        initBallAtRobotPosition();
        filterBallVelocity();
        updateExpectedBallEndPosition();
        updateBallAtRobotPosition();
    }

    void Ball::initBallAtRobotPosition() noexcept {
        auto previousWorld = World::instance()->getHistoryWorld(1);
        auto optionalPreviousBall = previousWorld->getBall();

        if (!optionalPreviousBall.has_value()) {
            return;
        }

        auto previousBall = optionalPreviousBall.value();

        if (position != Vector2() || previousBall->getPos() == Vector2()) {
            return;
        }

        // Current ball does not have a position, set it to the old pos
        auto rbtView = previousWorld.getRobotClosestToBall(us);
        if (rbtView.get()) {
            this->position = previousBall->getPos();
        }

    }

    void Ball::filterBallVelocity() noexcept {
        auto previousWorld = World::instance()->getHistoryWorld(1);
        auto optionalPreviousBall = previousWorld->getBall();

        if (!optionalPreviousBall.has_value()) {
            return;
        }

        auto oldBall = optionalPreviousBall.value();

        double velocityDifference = (velocity - oldBall->filteredVelocity).length() * ai::Constants::TICK_RATE();
        double factor = fmin(FILTER_MAX_FACTOR_FOR_VELOCITY,
                             velocityDifference * FILTER_MAX_FACTOR_FOR_VELOCITY / FILTER_VELOCITY_WITH_MAX_FACTOR);

        filteredVelocity = (oldBall->filteredVelocity * (1 - factor) + velocity * factor);

        /* When the Ball does not appear on the camera and later appears on the camera then the expected velocity suddenly
         * jumps to a very high value and so does the filtered velocity jump to a very high value. It will then take quite
         * long time for this filtered velocity to return to a more realistic velocity. To prevent this we will check if
         * the filtered velocity exceed some threshold value and if it does then we use the expected velocity as estimation
         * for the filtered velocity instead. */
        if (filteredVelocity.length2() > MAXIMUM_FILTER_VELOCITY) {
            filteredVelocity = velocity;
        }
    }

    void Ball::updateExpectedBallEndPosition() noexcept {
        auto previousWorld = World::instance()->getHistoryWorld(1);
        auto optionalPreviousBall = previousWorld->getBall();

        if (!optionalPreviousBall.has_value()) {
            return;
        }

        auto ball = optionalPreviousBall.value();

        double ballVelSquared = ball->getFilteredVelocity().length2();
        const double frictionCoefficient = ai::Constants::GRSIM() ? SIMULATION_FRICTION : REAL_FRICTION;

        expectedEndPosition =
                ball->getPos() + ball->filteredVelocity.stretchToLength(ballVelSquared / frictionCoefficient);

        namespace interface = ai::interface;

        //Visualize the Expected Ball End Position
        interface::Input::drawData(interface::Visual::BALL_DATA, {getExpectedEndPosition()},
                                   ai::Constants::BALL_COLOR(), -1, interface::Drawing::CIRCLES, 8, 8, 6);
        interface::Input::drawData(interface::Visual::BALL_DATA, {position, getExpectedEndPosition()},
                                   ai::Constants::BALL_COLOR(), -1, interface::Drawing::LINES_CONNECTED);
    }

    void Ball::updateBallAtRobotPosition() noexcept {
        if (isVisible()) {
            return;
        }

        auto world = World::instance()->getWorld();

        auto robotWithBall = world->whichRobotHasBall();
        if (!robotWithBall.get()) {
            return;
        }

        std::pair<uint8_t, Team> newRobotIdTeam = {robotWithBall->getId(), robotWithBall->getTeam()};
        view::RobotView newRobotWithBall{nullptr};
        for (auto robot : world->getUs()) {
            if (robot->getId() != newRobotIdTeam.first || robot->getTeam() != newRobotIdTeam.second) {
                continue;
            }
            newRobotWithBall = robot;

        }
        if (!newRobotWithBall.get()) {
            return;
        }
        double distanceInFrontOfRobot = ai::Constants::ROBOT_RADIUS() + ai::Constants::BALL_RADIUS();
        position = newRobotWithBall->getPos() + newRobotWithBall->getAngle().toVector2(distanceInFrontOfRobot);
    }

} // namespace rtt::world_new::ball
