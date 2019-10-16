#include "world/Ball.h"
#include "world/World.h"
#include "world/WorldData.h"
#include <interface/api/Input.h>
#include <control/ControlUtils.h>
#include <interface/api/Output.h>
#include <cmath>

namespace rtt {
namespace ai {
namespace world {

bool Ball::exists = false;

Ball::Ball()
        : expectedPosition(Vector2()), expectedVelocity(Vector2()), filteredVelocity(Vector2()),
          visibleByAnyCamera(false) {
}

Ball::Ball(const roboteam_proto::WorldBall &copy)
        : expectedPosition(copy.pos()), expectedVelocity(copy.vel()), filteredVelocity(copy.vel()),
          visibleByAnyCamera(copy.visible()) {
    exists = exists || copy.area() || Vector2(copy.pos()).isNotNaN();
    if (! exists) std::cout << "BallPtr message has existence = 0!!" << std::endl;
}

void Ball::updateBall(const BallPtr &oldBall, const WorldData &worldData) {
    initBallAtRobotPosition(*oldBall, worldData);
    filterBallVelocity(*oldBall, worldData);
    updateExpectedBallEndPosition(*oldBall, worldData);
    updateBallAtRobotPosition(*oldBall, worldData);
}

void Ball::initBallAtRobotPosition(const Ball &oldBall, const WorldData &worldData) {
    if (expectedPosition == Vector2() && oldBall.getPos() != Vector2()) {
        bool robotIsCloseToBall = false;
        for (const auto &robot : worldData.us) {
            if ((robot->pos - oldBall.getPos()).length() < THRESHOLD_ROBOT_CLOSE_TO_BALL) {
                robotIsCloseToBall = true;
                break;
            }
        }
        if (robotIsCloseToBall) {
            expectedPosition = oldBall.getPos();
        }
    }
}

void Ball::filterBallVelocity(Ball &oldBall, const WorldData &worldData) {
    double velocityDifference = (expectedVelocity - oldBall.filteredVelocity).length() * Constants::TICK_RATE();
    double factor = fmin(FILTER_MAX_FACTOR_FOR_VELOCITY,
                         velocityDifference * FILTER_MAX_FACTOR_FOR_VELOCITY / FILTER_VELOCITY_WITH_MAX_FACTOR);

    filteredVelocity = (oldBall.filteredVelocity * (1 - factor) + expectedVelocity * factor);

    /* When the Ball does not appear on the camera and later appears on the camera then the expected velocity suddenly
     * jumps to a very high value and so does the filtered velocity jump to a very high value. It will then take quite
     * long time for this filtered velocity to return to a more realistic velocity. To prevent this we will check if
     * the filtered velocity exceed some threshold value and if it does then we use the expected velocity as estimation
     * for the filtered velocity instead. */
    if (filteredVelocity.length2() > MAXIMUM_FILTER_VELOCITY) {
        filteredVelocity = expectedVelocity;
    }
}

void Ball::updateExpectedBallEndPosition(const Ball &oldBall, const WorldData &worldData) {
    auto &ball = worldData.ball;
    double ballVelSquared = ball->filteredVelocity.length2();
    const double frictionCoefficient = Constants::GRSIM() ? SIMULATION_FRICTION : REAL_FRICTION;

    expectedBallEndPosition = ball->getPos() + ball->filteredVelocity.stretchToLength(ballVelSquared / frictionCoefficient);

    //Visualize the Expected Ball End Position
    interface::Input::drawData(interface::Visual::BALL_DATA, {expectedBallEndPosition},
                               Constants::BALL_COLOR(), - 1,interface::Drawing::CIRCLES, 8, 8, 6);
    interface::Input::drawData(interface::Visual::BALL_DATA, {expectedPosition, expectedBallEndPosition},
                               Constants::BALL_COLOR(), - 1,interface::Drawing::LINES_CONNECTED);
}

void Ball::updateBallAtRobotPosition(const Ball &oldBall, const WorldData &worldData) {
    if (visibleByAnyCamera) {
        return;
    }
    RobotPtr robotWithBall = world->whichRobotHasBall();
    if (robotWithBall) {
        std::pair<int, Team> newRobotIdTeam = {robotWithBall->id, robotWithBall->team};
        RobotPtr newRobotWithBall;
        bool newRobotStillExistsInWorld = false;
        for (auto &robot : worldData.us) {
            if (robot->id == newRobotIdTeam.first && robot->team == newRobotIdTeam.second) {
                newRobotWithBall = robot;
                newRobotStillExistsInWorld = true;
            }
        }
        if (newRobotStillExistsInWorld) {
            double distanceInFrontOfRobot = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
            expectedPosition = newRobotWithBall->pos + newRobotWithBall->angle.toVector2(distanceInFrontOfRobot);
        }
    }
}

const Vector2 &Ball::getPos() const {
    return expectedPosition;
}

void Ball::setPos(const Vector2 &new_pos) {
    expectedPosition.x = new_pos.x;
    expectedPosition.y = new_pos.y;
}

const Vector2 &Ball::getVel() const {
    return expectedVelocity;
}

bool Ball::getVisible() {
    return visibleByAnyCamera;
}

void Ball::setVisible(bool visible) {
    visibleByAnyCamera = visible;
}

const Vector2 &Ball::getExpectedBallEndPosition() const {
    return expectedBallEndPosition;
}

} //world
} //ai
} //rtt