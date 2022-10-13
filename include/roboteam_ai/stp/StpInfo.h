//
// Created by john on 3/2/20.
//

#ifndef RTT_STPINFO_H
#define RTT_STPINFO_H

#include <optional>

#include "constants/GeneralizationConstants.h"
#include "utilities/StpInfoEnums.h"
#include "world/Field.h"
#include "world/views/BallView.hpp"
#include "world/views/RobotView.hpp"

namespace rtt::ai::stp {
namespace world = ::rtt::world;

/**
 * StpInfo bundles all info a robot could need in one struct
 * This data propagates all the way from plays down to skills
 */
struct StpInfo {
   public:
    const std::optional<world::view::RobotView>& getRobot() const { return robot; }
    void setRobot(const std::optional<world::view::RobotView>& robot) { this->robot = robot; }

    const std::optional<world::view::RobotView>& getEnemyRobot() const { return enemyRobot; }
    void setEnemyRobot(const std::optional<world::view::RobotView>& enemyRobot) { this->enemyRobot = enemyRobot; }

    const std::optional<world::Field>& getField() const { return field; }
    void setField(const std::optional<world::Field>& field) { this->field = field; }

    const std::optional<world::view::BallView>& getBall() const { return ball; }
    void setBall(const std::optional<world::view::BallView>& ball) { this->ball = ball; }

    const std::optional<Vector2>& getPositionToMoveTo() const { return positionToMoveTo; }
    void setPositionToMoveTo(const std::optional<Vector2>& position) { this->positionToMoveTo = position; }
    void setPositionToMoveTo(const std::optional<gen::ScoredPosition>& scoredPosition) {
        setRoleScore(scoredPosition->score);
        setPositionToMoveTo(scoredPosition->position);
    }

    const std::optional<Vector2>& getPositionToShootAt() const { return positionToShootAt; }
    void setPositionToShootAt(const std::optional<Vector2>& position) { this->positionToShootAt = position; }

    const std::optional<Vector2>& getPositionToDefend() const { return positionToDefend; }
    void setPositionToDefend(const std::optional<Vector2>& position) { this->positionToDefend = position; }

    double getKickChipVelocity() const { return kickChipVelocity; }
    void setKickChipVelocity(double kickChipVelocity) { this->kickChipVelocity = kickChipVelocity; }

    Angle getAngle() const { return angle; }
    void setAngle(double angle) { this->angle = Angle(angle); }

    int getDribblerSpeed() const { return dribblerSpeed; }
    void setDribblerSpeed(int dribblerSpeed) { this->dribblerSpeed = dribblerSpeed; }

    BlockDistance getBlockDistance() const { return blockDistance; }
    void setBlockDistance(BlockDistance blockDistance) { this->blockDistance = blockDistance; }

    ShotType getShotType() const { return shotType; }
    void setShotType(ShotType shotType) { this->shotType = shotType; }

    void setKickOrChip(const std::optional<KickOrChip>& kickOrChip) { StpInfo::kickOrChip = kickOrChip; }

    world::World* getCurrentWorld() const { return currentWorld; }
    /// This function is used in a lambda, [[maybe_unused]] is to suppress 'unused' warnings
    [[maybe_unused]] void setCurrentWorld(world::World* world) { currentWorld = world; }

    const std::optional<PIDType>& getPidType() const { return PidType; }
    void setPidType(const std::optional<PIDType>& pidType) { PidType = pidType; }

    void setRoleScore(const std::optional<uint8_t>& RoleScore) { roleScore = RoleScore; }

    double getMaxRobotVelocity() const { return maxRobotVelocity; }
    void setMaxRobotVelocity(double maxVelocity) { maxRobotVelocity = maxVelocity; }

    std::string getRoleName() const { return roleName; }
    void setRoleName(std::string name) { roleName = name; }

    AvoidObjects getObjectsToAvoid() const { return avoidObjects; }

    void setShouldAvoidDefenseArea(bool shouldAvoidDefenseArea) { avoidObjects.shouldAvoidDefenseArea = shouldAvoidDefenseArea; }

    void setShouldAvoidBall(bool shouldAvoidBall) { avoidObjects.shouldAvoidBall = shouldAvoidBall; }

    void setShouldAvoidOutOfField(bool shouldAvoidOutOfField) { avoidObjects.shouldAvoidOutOfField = shouldAvoidOutOfField; }

    void setShouldAvoidOurRobots(bool shouldAvoidOurRobots) { avoidObjects.shouldAvoidOurRobots = shouldAvoidOurRobots; }

    void setShouldAvoidTheirRobots(bool shouldAvoidTheirRobots) { avoidObjects.shouldAvoidTheirRobots = shouldAvoidTheirRobots; }

   private:
    /**
     * Current world pointer
     */
    world::World* currentWorld;

    /**
     * Robot this tactic applies to
     */
    std::optional<world::view::RobotView> robot;

    /**
     * EnemyRobot this tactic applies to
     */
    std::optional<world::view::RobotView> enemyRobot;

    /**
     * Field
     */
    std::optional<world::Field> field;

    /**
     * View to the ball in the world this tactic executes on
     */
    std::optional<world::view::BallView> ball;

    /**
     * Position to move to
     */
    std::optional<Vector2> positionToMoveTo;

    /**
     * Position to kick or chip at
     */
    std::optional<Vector2> positionToShootAt;

    /**
     * Position to defend
     */
    std::optional<Vector2> positionToDefend;

    /**
     * Velocity of the kick/chip
     */
    double kickChipVelocity = 0.0;

    /**
     * Type of the kick/chip
     */
    ShotType shotType{};

    /**
     * Reference angle of the robot
     */
    Angle angle = Angle(0.0);

    /**
     * Speed of the dribbler in %
     */
    int dribblerSpeed = 0;

    /**
     * The distance of our robot to a to-be-blocked target
     */
    BlockDistance blockDistance = BlockDistance::CLOSE;

    /**
     * Set the shot to be a kick or chip
     */
    std::optional<KickOrChip> kickOrChip;

    /**
     * Enum for deciding which PID should be chosen
     */
    std::optional<PIDType> PidType{PIDType::DEFAULT};

    /**
     * Optional roleScore value to be used in play score determination
     */
    std::optional<uint8_t> roleScore;

    /**
     * The maximum velocity the robot is allowed to have
     */
    double maxRobotVelocity;

    /**
     * The name of the role associated with this StpInfo
     */
    std::string roleName;

    /**
     * Specify what objects the robot should avoid
     */
    AvoidObjects avoidObjects;
};

/**
 * Util operator<< that allows us to print the status enum
 */
[[maybe_unused]] static std::ostream& operator<<(std::ostream& os, Status status) {
    switch (status) {
        case Status::Waiting:
            return os << "Status::Waiting";
        case Status::Success:
            return os << "Status::Success";
        case Status::Failure:
            return os << "Status::Failure";
        case Status::Running:
            return os << "Status::Running";
        default:
            return os << "INVALID STATUS";
    }
}
}  // namespace rtt::ai::stp

#endif  // RTT_STPINFO_H
