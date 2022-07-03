//
// Created by john on 12/16/19.
//

#include "world/Robot.hpp"

#include "utilities/Constants.h"
#include "world/World.hpp"

namespace rtt::world::robot {
Robot::Robot(const proto::WorldRobot &copy, rtt::world::Team team, std::optional<view::BallView> ball,
             unsigned char dribblerState, unsigned long worldNumber)
    : id{static_cast<int>(copy.id())},
      team{team},
      pos{copy.pos().x(), copy.pos().y()},
      vel{copy.vel().x(), copy.vel().y()},
      angle{copy.angle()},
      distanceToBall{-1.0},
      lastUpdatedWorldNumber{worldNumber},
      angularVelocity{copy.w()},
      dribblerState{dribblerState} {
    if (id < 16) {
        workingDribbler = ai::Constants::ROBOT_HAS_WORKING_DRIBBLER(id);
        workingBallSensor = ai::Constants::ROBOT_HAS_WORKING_BALL_SENSOR(id);
    }

    if (team == Team::us) {
        if(copy.has_feedbackinfo()){
            updateFromFeedback(copy.feedbackinfo());
        }
    }

    if (ball.has_value()) {
        setDistanceToBall(pos.dist((*ball)->position));
        auto angleRobotToBall = ((*ball)->position - pos).angle();
        setAngleDiffToBall(angle.shortestAngleDiff(Angle(angleRobotToBall)));

        // For our own robots in the sim, we only use the ballSensor, since its more accurate
        if (!(team == Team::us && SETTINGS.getRobotHubMode() == Settings::RobotHubMode::SIMULATOR)) {
            // If the ball is not visible, we should go closer to the ball before thinking we have it, for safety (since we can't actually see if we have the ball or not)
            auto hasBallDist = ball->get()->visible ? ai::Constants::HAS_BALL_DISTANCE() : ai::Constants::HAS_BALL_DISTANCE() * 0.75;
            auto hasBallAccordingToVision = distanceToBall < hasBallDist && angleDiffToBall < ai::Constants::HAS_BALL_ANGLE();

            // Update the hasBall map- if vision thinks we have the ball, add 1, otherwise subtract 1
            hasBallUpdateMap[id] += (hasBallAccordingToVision ? 1 : -1);
            // If we have a ballsensor, also use its information to update the hasBall map
            if (workingBallSensor) hasBallUpdateMap[id] += (ballSensorSeesBall() ? 1 : -1);
        } else {
            // In the sim, for our team, we only use the ballsensor (since its very accurate)
            if (workingBallSensor) hasBallUpdateMap[id] += (ballSensorSeesBall() ? 2 : -2);
            else RTT_WARNING("Ball Sensor not working, robot does not know whether it has ball!");
        }

        // Make sure the value does not get too large/small
        hasBallUpdateMap[id] = std::clamp(hasBallUpdateMap[id], 0, 12);
        setHasBall(hasBallUpdateMap[id] > 8);

        /// TODO: There's some magic numbers here: the max value at which we clamped could be higher/lower, and the cutoff for saying we have the ball could be different
        /// Furthermore, it might be good to have 2 decision boundaries instead of one. I.e., if we have the ball, only change that if it goes < 3. If we do not have the ball, switch when > 8
    }
}

int Robot::getId() const noexcept { return id; }

void Robot::setId(int _id) noexcept { Robot::id = _id; }

Team Robot::getTeam() const noexcept { return team; }

void Robot::setTeam(Team _team) noexcept { Robot::team = _team; }

const Vector2 &Robot::getPos() const noexcept { return pos; }

void Robot::setPos(const Vector2 &_pos) noexcept { Robot::pos = _pos; }

const Vector2 &Robot::getVel() const noexcept { return vel; }

void Robot::setVel(const Vector2 &_vel) noexcept { Robot::vel = _vel; }

const Angle &Robot::getAngle() const noexcept { return angle; }

void Robot::setAngle(const Angle &_angle) noexcept { Robot::angle = _angle; }

double Robot::getAngularVelocity() const noexcept { return angularVelocity; }

void Robot::setAngularVelocity(double _angularVelocity) noexcept { Robot::angularVelocity = _angularVelocity; }

bool Robot::isBatteryLow() const noexcept { return batteryLow; }

void Robot::setBatteryLow(bool _batteryLow) noexcept { Robot::batteryLow = _batteryLow; }

unsigned char Robot::getDribblerState() const noexcept { return dribblerState; }

void Robot::setDribblerState(unsigned char _dribblerState) noexcept { Robot::dribblerState = _dribblerState; }

unsigned char Robot::getPreviousDribblerState() const noexcept { return previousDribblerState; }

void Robot::setPreviousDribblerState(unsigned char _previousDribblerState) noexcept { Robot::previousDribblerState = _previousDribblerState; }

double Robot::getTimeDribblerChanged() const noexcept { return timeDribblerChanged; }

void Robot::setTimeDribblerChanged(double _timeDribblerChanged) noexcept { Robot::timeDribblerChanged = _timeDribblerChanged; }

bool Robot::isWorkingDribbler() const noexcept { return workingDribbler; }

void Robot::setWorkingDribbler(bool _workingDribbler) noexcept { Robot::workingDribbler = _workingDribbler; }

bool Robot::isWorkingBallSensor() const noexcept { return workingBallSensor; }

void Robot::setWorkingBallSensor(bool _workingBallSensor) noexcept { Robot::workingBallSensor = _workingBallSensor; }

bool Robot::ballSensorSeesBall() const noexcept { return seesBall; }

void Robot::setBallSensorSeesBall(bool _seesBall) noexcept { Robot::seesBall = _seesBall; }

void Robot::setHasBall(bool _hasBall) noexcept { Robot::robotHasBall = _hasBall; }

bool Robot::hasBall() const noexcept { return robotHasBall; }

float Robot::getBallPosBallSensor() const noexcept { return ballPos; }

void Robot::setBallPosBallSensor(float _ballPos) noexcept { Robot::ballPos = _ballPos; }

const Vector2 &Robot::getPidPreviousVel() const noexcept { return pidPreviousVel; }

void Robot::setPidPreviousVel(const Vector2 &_pidPreviousVel) noexcept { Robot::pidPreviousVel = _pidPreviousVel; }

double Robot::getDistanceToBall() const noexcept { return distanceToBall; }

void Robot::setDistanceToBall(double _distanceToBall) noexcept { Robot::distanceToBall = _distanceToBall; }

double Robot::getAngleDiffToBall() const noexcept { return angleDiffToBall; }

void Robot::setAngleDiffToBall(double _angleDiffToBall) noexcept { Robot::angleDiffToBall = _angleDiffToBall; }

unsigned long Robot::getLastUpdatedWorldNumber() const noexcept { return lastUpdatedWorldNumber; }

void Robot::setLastUpdatedWorldNumber(unsigned long _lastUpdatedWorldNumber) noexcept { Robot::lastUpdatedWorldNumber = _lastUpdatedWorldNumber; }

void Robot::updateFromFeedback(const proto::RobotProcessedFeedback &feedback) noexcept {
    //TODO: add processing of more of the fields of feedback
    if (ai::Constants::FEEDBACK_ENABLED()) {
        setWorkingBallSensor(feedback.ball_sensor_is_working());
        setBatteryLow(feedback.battery_level() < 22);  // TODO: Define what is considered a 'low' voltage
        setBallSensorSeesBall(feedback.has_ball());
        setBallPosBallSensor(feedback.ball_position());
    }
}
}  // namespace rtt::world::robot
