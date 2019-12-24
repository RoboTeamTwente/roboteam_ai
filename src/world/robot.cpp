//
// Created by john on 12/16/19.
//

#include "roboteam_ai/utilities/Constants.h"
#include "roboteam_world/world/robot.hpp"

#include <cassert>

namespace rtt::world::robot {
    Robot::Robot(std::unordered_map<uint8_t, proto::RobotFeedback> &feedback,
                 const proto::WorldRobot &copy, rtt::world::team::Team team,
                 unsigned char genevaState,
                 unsigned char dribblerState, unsigned long worldNumber)
            : team{team},
              genevaState{genevaState},
              distanceToBall{-1.0},
              lastUpdatedWorldNumber{worldNumber},
              dribblerState{dribblerState},
              id{copy.id()},
              angle{copy.angle()},
              pos{copy.pos()},
              vel{copy.vel()},
              angularVelocity{copy.w()} {
        if (id < 16) {
            workingGeneva = ai::Constants::ROBOT_HAS_WORKING_GENEVA(id);
            workingDribbler = ai::Constants::ROBOT_HAS_WORKING_DRIBBLER(id);
            workingBallSensor = ai::Constants::ROBOT_HAS_WORKING_BALL_SENSOR(id);
        } else {
            std::cerr << "Warning: Creating robot with id = " << id << std::endl;
            assert(false);
        }

        if (feedback.find(id) != feedback.end()) {
            updateFromFeedback(feedback[id]);
        }

        resetShotController();
        resetNumTreePosControl();
        resetBasicPosControl();
        resetBallHandlePosControl();
    }

    uint32_t Robot::getId() const noexcept {
        return id;
    }

    void Robot::setId(uint32_t _id) noexcept {
        Robot::id = _id;
    }

    team::Team Robot::getTeam() const noexcept {
        return team;
    }

    void Robot::setTeam(team::Team _team) noexcept {
        Robot::team = _team;
    }

    uint8_t Robot::getGenevaState() const noexcept {
        return genevaState;
    }

    void Robot::setGenevaState(uint8_t _genevaState) noexcept {
        Robot::genevaState = _genevaState;
    }

    uint8_t Robot::getPreviousGenevaState() const noexcept {
        return previousGenevaState;
    }

    void Robot::setPreviousGenevaState(uint8_t _previousGenevaState) noexcept {
        Robot::previousGenevaState = _previousGenevaState;
    }

    const Vector2 &Robot::getPos() const noexcept {
        return pos;
    }

    void Robot::setPos(const Vector2 &_pos) noexcept {
        Robot::pos = _pos;
    }

    const Vector2 &Robot::getVel() const noexcept {
        return vel;
    }

    void Robot::setVel(const Vector2 &_vel) noexcept {
        Robot::vel = _vel;
    }

    const Angle &Robot::getAngle() const noexcept {
        return angle;
    }

    void Robot::setAngle(const Angle &_angle) noexcept {
        Robot::angle = _angle;
    }

    double Robot::getAngularVelocity() const noexcept {
        return angularVelocity;
    }

    void Robot::setAngularVelocity(double _angularVelocity) noexcept {
        Robot::angularVelocity = _angularVelocity;
    }

    double Robot::getTimeGenevaChanged() const noexcept {
        return timeGenevaChanged;
    }

    void Robot::setTimeGenevaChanged(double _timeGenevaChanged) noexcept {
        Robot::timeGenevaChanged = _timeGenevaChanged;
    }

    bool Robot::isWorkingGeneva() const noexcept {
        return workingGeneva;
    }

    void Robot::setWorkingGeneva(bool _workingGeneva) noexcept {
        Robot::workingGeneva = _workingGeneva;
    }

    bool Robot::isBatteryLow() const noexcept {
        return batteryLow;
    }

    void Robot::setBatteryLow(bool _batteryLow) noexcept {
        Robot::batteryLow = _batteryLow;
    }

    unsigned char Robot::getDribblerState() const noexcept {
        return dribblerState;
    }

    void Robot::setDribblerState(unsigned char _dribblerState) noexcept {
        Robot::dribblerState = _dribblerState;
    }

    unsigned char Robot::getPreviousDribblerState() const noexcept {
        return previousDribblerState;
    }

    void Robot::setPreviousDribblerState(unsigned char _previousDribblerState) noexcept {
        Robot::previousDribblerState = _previousDribblerState;
    }

    double Robot::getTimeDribblerChanged() const noexcept {
        return timeDribblerChanged;
    }

    void Robot::setTimeDribblerChanged(double _timeDribblerChanged) noexcept {
        Robot::timeDribblerChanged = _timeDribblerChanged;
    }

    bool Robot::isWorkingDribbler() const noexcept {
        return workingDribbler;
    }

    void Robot::setWorkingDribbler(bool _workingDribbler) noexcept {
        Robot::workingDribbler = _workingDribbler;
    }

    bool Robot::isWorkingBallSensor() const noexcept {
        return workingBallSensor;
    }

    void Robot::setWorkingBallSensor(bool _workingBallSensor) noexcept {
        Robot::workingBallSensor = _workingBallSensor;
    }

    void Robot::resetShotController() noexcept {
        shotController = std::make_unique<ai::control::ShotController>();
    }

    void Robot::resetNumTreePosControl() noexcept {
        numTreePosControl = std::make_unique<ai::control::NumTreePosControl>();
    }

    void Robot::resetBasicPosControl() noexcept {
        basicPosControl = std::make_unique<ai::control::BasicPosControl>();
    }

    void Robot::resetBallHandlePosControl() noexcept {
        ballHandlePosControl = std::make_unique<ai::control::BallHandlePosControl>();
    }

    ai::control::ShotController *Robot::getShotController() const noexcept {
        return shotController.get();
    }

    ai::control::NumTreePosControl *Robot::getNumTreePosControl() const noexcept {
        return numTreePosControl.get();
    }

    ai::control::BasicPosControl *Robot::getBasicPosControl() const noexcept {
        return basicPosControl.get();
    }

    ai::control::BallHandlePosControl *Robot::getBallHandlePosControl() const noexcept {
        return ballHandlePosControl.get();
    }

    const Vector2 &Robot::getPidPreviousVel() const noexcept {
        return pidPreviousVel;
    }

    void Robot::setPidPreviousVel(const Vector2 &_pidPreviousVel) noexcept {
        Robot::pidPreviousVel = _pidPreviousVel;
    }

    double Robot::getDistanceToBall() const noexcept {
        return distanceToBall;
    }

    void Robot::setDistanceToBall(double _distanceToBall) noexcept {
        Robot::distanceToBall = _distanceToBall;
    }

    bool Robot::isIHaveBall() const noexcept {
        return iHaveBall;
    }

    void Robot::setIHaveBall(bool _iHaveBall) noexcept {
        Robot::iHaveBall = _iHaveBall;
    }

    unsigned long Robot::getLastUpdatedWorldNumber() const noexcept {
        return lastUpdatedWorldNumber;
    }

    void Robot::setLastUpdatedWorldNumber(unsigned long _lastUpdatedWorldNumber) noexcept {
        Robot::lastUpdatedWorldNumber = _lastUpdatedWorldNumber;
    }

    void Robot::updateFromFeedback(proto::RobotFeedback &feedback) noexcept {
        if (ai::Constants::FEEDBACK_ENABLED()) {
            setWorkingGeneva(feedback.genevaisworking());
            setWorkingBallSensor(feedback.ballsensorisworking());
            setBatteryLow(feedback.batterylow());
        }
    }
} // namespace rtt::world::robot