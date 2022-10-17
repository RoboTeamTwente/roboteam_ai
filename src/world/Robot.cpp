//
// Created by john on 12/16/19.
//

#include "world/Robot.hpp"

#include "utilities/Constants.h"
#include "world/World.hpp"

namespace rtt::world::robot {
Robot::Robot(const proto::WorldRobot &copy, rtt::world::Team team, std::optional<view::BallView> ball, unsigned char dribblerState, unsigned long worldNumber)
    : id{static_cast<int>(copy.id())},
      team{team},
      pos{copy.pos().x(), copy.pos().y()},
      vel{copy.vel().x(), copy.vel().y()},
      angle{copy.angle()},
      distanceToBall{-1.0},
      angularVelocity{copy.w()} {
    if (id < 16) {
        workingDribbler = ai::Constants::ROBOT_HAS_WORKING_DRIBBLER(id);
        workingBallSensor = ai::Constants::ROBOT_HAS_WORKING_BALL_SENSOR(id);
    }

    if (ball.has_value()) {
        setDistanceToBall(pos.dist((*ball)->position));
        auto angleRobotToBall = ((*ball)->position - pos).angle();
        setAngleDiffToBall(angle.shortestAngleDiff(Angle(angleRobotToBall)));
    }

    if (team == Team::us) {
        if (copy.has_feedbackinfo()) {
            updateFromFeedback(copy.feedbackinfo());
        }
        updateHasBallMap(ball);
    } else {
        auto hasBallAccordingToVision = distanceToBall < ai::Constants::HAS_BALL_DISTANCE() && angleDiffToBall < ai::Constants::HAS_BALL_ANGLE();
        setHasBall(hasBallAccordingToVision);
    }
}

int Robot::getId() const noexcept { return id; }

Team Robot::getTeam() const noexcept { return team; }

const Vector2 &Robot::getPos() const noexcept { return pos; }

const Vector2 &Robot::getVel() const noexcept { return vel; }

const Angle &Robot::getAngle() const noexcept { return angle; }

void Robot::setAngle(const Angle &_angle) noexcept { Robot::angle = _angle; }

double Robot::getAngularVelocity() const noexcept { return angularVelocity; }

bool Robot::isBatteryLow() const noexcept { return batteryLow; }

void Robot::setBatteryLow(bool _batteryLow) noexcept { Robot::batteryLow = _batteryLow; }

bool Robot::isWorkingDribbler() const noexcept { return workingDribbler; }

bool Robot::isWorkingBallSensor() const noexcept { return workingBallSensor; }

void Robot::setWorkingBallSensor(bool _workingBallSensor) noexcept { Robot::workingBallSensor = _workingBallSensor; }

void Robot::setBallSensorSeesBall(bool _seesBall) noexcept { ballSensorSeesBall = _seesBall; }

void Robot::setDribblerSeesBall(bool _seesBall) noexcept { dribblerSeesBall = _seesBall; }

void Robot::setHasBall(bool _hasBall) noexcept { Robot::robotHasBall = _hasBall; }

bool Robot::hasBall() const noexcept { return robotHasBall; }

void Robot::setBallPosBallSensor(float _ballPos) noexcept { Robot::ballPos = _ballPos; }

double Robot::getDistanceToBall() const noexcept { return distanceToBall; }

void Robot::setDistanceToBall(double _distanceToBall) noexcept { Robot::distanceToBall = _distanceToBall; }

double Robot::getAngleDiffToBall() const noexcept { return angleDiffToBall; }

void Robot::setAngleDiffToBall(double _angleDiffToBall) noexcept { Robot::angleDiffToBall = _angleDiffToBall; }

void Robot::updateFromFeedback(const proto::RobotProcessedFeedback &feedback) noexcept {
    // TODO: add processing of more of the fields of feedback
    if (ai::Constants::FEEDBACK_ENABLED()) {
        setWorkingBallSensor(feedback.ball_sensor_is_working());
        setBatteryLow(feedback.battery_level() < 22);  // TODO: Define what is considered a 'low' voltage
        setBallSensorSeesBall(feedback.ball_sensor_sees_ball());
        setDribblerSeesBall(feedback.dribbler_sees_ball());
        setBallPosBallSensor(feedback.ball_position());
    }
}

void Robot::updateHasBallMap(std::optional<view::BallView> &ball) {
    if (!ball) return;

    // When doing free kicks, we have to immediately kick the ball, hence, we only check for 1 tick
    // TODO: this is a bit of a hacky way to avoid double touch fouls. Figuring out a better way to do this would be nice
    if (ai::GameStateManager::getCurrentGameState().getStrategyName() == "free_kick_us" || ai::GameStateManager::getCurrentGameState().getStrategyName() == "kickoff_us") {
        auto hasBallAccordingToVision = distanceToBall < ai::Constants::HAS_BALL_DISTANCE() * 0.9 && angleDiffToBall < ai::Constants::HAS_BALL_ANGLE();
        if (hasBallAccordingToVision || dribblerSeesBall) setHasBall(true);
        return;
    }

    // On the field, use data from the dribbler and vision to determine if we have the ball
    if (SETTINGS.getRobotHubMode() == Settings::RobotHubMode::BASESTATION) {
        // If the ball is not visible, we should go closer to the ball before thinking we have it, for safety (since we can't actually see if we have the ball or not)
        auto hasBallDist = ball->get()->visible ? ai::Constants::HAS_BALL_DISTANCE() : ai::Constants::HAS_BALL_DISTANCE() * 0.75;
        auto hasBallAccordingToVision = distanceToBall < hasBallDist && angleDiffToBall < ai::Constants::HAS_BALL_ANGLE();

        // Increase the hasBall score depending on how sure we are that we have the ball
        if (hasBallAccordingToVision && dribblerSeesBall)
            hasBallUpdateMap[id].score += 2;
        else if (hasBallAccordingToVision && !dribblerSeesBall)
            hasBallUpdateMap[id].score += 1;
        else if (!hasBallAccordingToVision && dribblerSeesBall && distanceToBall < ai::Constants::HAS_BALL_DISTANCE() * 1.5 &&
                 angleDiffToBall < ai::Constants::HAS_BALL_ANGLE() * 1.5)
            hasBallUpdateMap[id].score += 1;
        else
            hasBallUpdateMap[id].score -= 2;

        // TODO when we have working ballsensors: Use the ballsensor as well to determine if we have the ball
        // if (workingBallSensor) hasBallUpdateMap[id].score += (ballSensorSeesBall ? 1 : -1);
    } else {
        // In the sim, for our team, we only use the ballsensor (since its very accurate)
        hasBallUpdateMap[id].score += (ballSensorSeesBall ? 2 : -2);
    }

    // Make sure the value does not get too large/small
    hasBallUpdateMap[id].score = std::clamp(hasBallUpdateMap[id].score, 0, 25);

    // If we previously had the ball, we do not have the ball if the score gets below 4
    if (hasBallUpdateMap[id].hasBall && hasBallUpdateMap[id].score < 4) hasBallUpdateMap[id].hasBall = false;
    // If we did not have the ball yet, we have the ball if the score gets over 20

    // Temporary fix increasing sensitivity to hasBall scoring during testing. Needs to be extracted and fine tuned.
    else if (hasBallUpdateMap[id].score > 0)
        hasBallUpdateMap[id].hasBall = true;

    setHasBall(hasBallUpdateMap[id].hasBall);

    /// TODO: There's some magic numbers here: the max value at which we clamped could be higher/lower, and the cutoff for saying we have the ball could be different
    /// These values should be tuned further to balance speed and reliability
}
}  // namespace rtt::world::robot
