//
// Created by john on 12/16/19.
//

#ifndef RTT_ROBOT_HPP
#define RTT_ROBOT_HPP

#include <proto/RobotFeedback.pb.h>
#include <proto/WorldRobot.pb.h>

#include <optional>

#include "Team.hpp"
#include "roboteam_utils/Angle.h"
#include "world/views/BallView.hpp"

namespace rtt::world::robot {


struct hasBallInfo{
    bool hasBall;
    int score;
};

/**
 * robot still changes:
 *  battery
 *  spinner speed
 */
class Robot {
   private:
    int id;
    Team team;

    Vector2 pos;
    Vector2 vel;
    Angle angle;

    double distanceToBall;
    double angleDiffToBall;

    double angularVelocity;
    bool batteryLow{false};

    bool workingDribbler;
    bool workingBallSensor{};

    bool ballSensorSeesBall{}; // This value might be incorrect. Use robotHasBall!
    float ballPos{};
    bool dribblerSeesBall{}; // This value might be incorrect. Use robotHasBall!
    bool robotHasBall{};

    // map that stores a score that indicates how likely we think it is that each robot has the ball
    static inline std::unordered_map<int, hasBallInfo> hasBallUpdateMap;

   private:
    void updateFromFeedback(const proto::RobotProcessedFeedback &feedback) noexcept;

    void updateHasBallMap(std::optional<view::BallView>& ball);

    void setAngle(const Angle &angle) noexcept;

    void setBatteryLow(bool batteryLow) noexcept;

    void setWorkingBallSensor(bool workingBallSensor) noexcept;

    void setDistanceToBall(double distanceToBall) noexcept;

    void setAngleDiffToBall(double _angleDiffToBall) noexcept;

    void setBallSensorSeesBall(bool _seesBall) noexcept;

    void setDribblerSeesBall(bool _seesBall) noexcept;

    void setHasBall(bool _hasBall) noexcept;

    void setBallPosBallSensor(float _ballPos) noexcept;

   public:
    [[nodiscard]] int getId() const noexcept;

    [[nodiscard]] Team getTeam() const noexcept;

    [[nodiscard]] const Vector2 &getPos() const noexcept;

    [[nodiscard]] const Vector2 &getVel() const noexcept;

    [[nodiscard]] const Angle &getAngle() const noexcept;

    [[nodiscard]] double getAngularVelocity() const noexcept;

    [[nodiscard]] bool isBatteryLow() const noexcept;

    [[nodiscard]] bool isWorkingDribbler() const noexcept;

    [[nodiscard]] bool isWorkingBallSensor() const noexcept;

    [[nodiscard]] bool hasBall() const noexcept;

    [[nodiscard]] double getDistanceToBall() const noexcept;

    [[nodiscard]] double getAngleDiffToBall() const noexcept;

   public:
    explicit Robot(const proto::WorldRobot &copy, Team team = both,
                   std::optional<rtt::world::view::BallView> ball = std::nullopt, unsigned char dribblerState = 0, unsigned long worldNumber = 0);

    Robot &operator=(Robot const &) = default;

    Robot(Robot const &) = default;

    Robot &operator=(Robot &&) = default;

    Robot(Robot &&) = default;
};
}  // namespace rtt::world::robot

#endif  // RTT_ROBOT_HPP
