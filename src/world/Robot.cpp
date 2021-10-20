//
// Created by john on 12/16/19.
//

#include "world/Robot.hpp"

#include "utilities/Constants.h"
#include "world/World.hpp"

namespace rtt::world::robot {
Robot::Robot(std::unordered_map<uint8_t, proto::RobotFeedback> &feedback, const proto::WorldRobot &copy, rtt::world::Team team, std::optional<view::BallView> ball,
             unsigned char dribblerState, unsigned long worldNumber)
    : team{team},
      distanceToBall{-1.0},
      lastUpdatedWorldNumber{worldNumber},
      dribblerState{dribblerState},
      id{copy.id()},
      angle{copy.angle()},
      pos{copy.pos()},
      vel{copy.vel()},
      angularVelocity{copy.w()} {
    if (id < 16) {
        workingDribbler = ai::Constants::ROBOT_HAS_WORKING_DRIBBLER(id);
        workingBallSensor = ai::Constants::ROBOT_HAS_WORKING_BALL_SENSOR(id);
    }

    if (feedback.find(id) != feedback.end()) {
        updateFromFeedback(feedback[id]);
    }

    if (ball.has_value()) {
        setDistanceToBall(pos.dist((*ball)->getPos()));
        auto angleRobotToBall = ((*ball)->getPos() - pos).angle();
        setAngleDiffToBall(angle.shortestAngleDiff(Angle(angleRobotToBall)));
    }
}

uint32_t Robot::getId() const noexcept { return id; }

void Robot::setId(uint32_t _id) noexcept { Robot::id = _id; }

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

void Robot::updateFromFeedback(proto::RobotFeedback &feedback) noexcept {
    if (ai::Constants::FEEDBACK_ENABLED()) {
        setWorkingBallSensor(feedback.ballsensorisworking());
        setBatteryLow(feedback.batterylow());
        setBallSensorSeesBall(feedback.hasball());
        setBallPosBallSensor(feedback.ballpos());
    }
}
}  // namespace rtt::world::robot
