//
// Created by john on 12/16/19.
//

#ifndef RTT_ROBOT_HPP
#define RTT_ROBOT_HPP

#include <roboteam_proto/RobotFeedback.pb.h>

#include "RobotType.hpp"
#include "roboteam_proto/WorldRobot.pb.h"
#include "roboteam_utils/Angle.h"
#include "world_new/Team.hpp"
#include "world_new/views/BallView.hpp"

namespace rtt::ai::control {
class ShotController;
class NumTreePosControl;
class BallHandlePosControl;
class BasicPosControl;
}  // namespace rtt::ai::control

namespace rtt::world_new::robot {

/**
 * Geneva driver gone
 * 30 / 50 watt motor flag
 *
 * robot still changes:
 *  battery
 *  spinner speed
 */
class Robot {
   private:
    uint32_t id;
    Team team;
    RobotType type = RobotType::THIRTY_WATT;

    Vector2 pos;
    Vector2 vel;
    Angle angle;
    Vector2 pidPreviousVel;

    double distanceToBall;
    unsigned long lastUpdatedWorldNumber = 0;

    double angularVelocity;
    bool batteryLow{false};

    unsigned char dribblerState = 0;
    unsigned char previousDribblerState = 0;

    double timeDribblerChanged = 0;
    constexpr static double timeToChangeOneDribblerLevel = 0.18;
    bool workingDribbler;
    bool workingBallSensor;

    bool hasBall;
    float ballPos;

   private:
    void updateFromFeedback(proto::RobotFeedback &feedback) noexcept;

    void setRobotType(RobotType type) noexcept;

    void setId(uint32_t id) noexcept;

    void setTeam(Team team) noexcept;

    void setPos(const Vector2 &pos) noexcept;

    void setVel(const Vector2 &vel) noexcept;

    void setAngle(const Angle &angle) noexcept;

    void setAngularVelocity(double angularVelocity) noexcept;

    void setBatteryLow(bool batteryLow) noexcept;

    void setDribblerState(unsigned char dribblerState) noexcept;

    void setPreviousDribblerState(unsigned char previousDribblerState) noexcept;

    void setTimeDribblerChanged(double timeDribblerChanged) noexcept;

    void setWorkingDribbler(bool workingDribbler) noexcept;

    void setWorkingBallSensor(bool workingBallSensor) noexcept;

    void setDistanceToBall(double distanceToBall) noexcept;

    void setHasBallBallSensor(bool hasBall) noexcept;

    void setBallPosBallSensor(float ballPos) noexcept;

    void setIHaveBall(bool iHaveBall) noexcept;

    void setLastUpdatedWorldNumber(unsigned long lastUpdatedWorldNumber) noexcept;

    void setPidPreviousVel(const Vector2 &pidPreviousVel) noexcept;

   public:
    [[nodiscard]] uint32_t getId() const noexcept;

    [[nodiscard]] RobotType getRobotType() const noexcept;

    [[nodiscard]] Team getTeam() const noexcept;

    [[nodiscard]] const Vector2 &getPos() const noexcept;

    [[nodiscard]] const Vector2 &getVel() const noexcept;

    [[nodiscard]] const Angle &getAngle() const noexcept;

    [[nodiscard]] double getAngularVelocity() const noexcept;

    [[nodiscard]] bool isBatteryLow() const noexcept;

    [[nodiscard]] bool isFiftyWatt() const noexcept;

    [[nodiscard]] bool isThirtyWatt() const noexcept;

    [[nodiscard]] unsigned char getDribblerState() const noexcept;

    [[nodiscard]] unsigned char getPreviousDribblerState() const noexcept;

    [[nodiscard]] double getTimeDribblerChanged() const noexcept;

    [[nodiscard]] bool isWorkingDribbler() const noexcept;

    [[nodiscard]] bool isWorkingBallSensor() const noexcept;

    [[nodiscard]] bool hasBallBallSensor() const noexcept;

    [[nodiscard]] float getBallPosBallSensor() const noexcept;

    [[nodiscard]] ai::control::ShotController *getShotController() const noexcept;

    [[nodiscard]] ai::control::NumTreePosControl *getNumTreePosControl() const noexcept;

    [[nodiscard]] ai::control::BasicPosControl *getBasicPosControl() const noexcept;

    [[nodiscard]] ai::control::BallHandlePosControl *getBallHandlePosControl() const noexcept;

    [[nodiscard]] const Vector2 &getPidPreviousVel() const noexcept;

    [[nodiscard]] double getDistanceToBall() const noexcept;

    [[nodiscard]] bool isIHaveBall() const noexcept;

    [[nodiscard]] unsigned long getLastUpdatedWorldNumber() const noexcept;

    void resetShotController() const noexcept;

    void resetNumTreePosControl() const noexcept;

    void resetBasicPosControl() const noexcept;

    void resetBallHandlePosControl() const noexcept;

   public:
    explicit Robot(std::unordered_map<uint8_t, proto::RobotFeedback> &feedback, const proto::WorldRobot &copy, Team team = both,
                   std::optional<rtt::world_new::view::BallView> ball = std::nullopt, unsigned char dribblerState = 0, unsigned long worldNumber = 0);

    Robot &operator=(Robot const &) = default;

    Robot(Robot const &) = default;

    Robot &operator=(Robot &&) = default;

    Robot(Robot &&) = default;
};
}  // namespace rtt::world_new::robot

#endif  // RTT_ROBOT_HPP
