//
// Created by john on 3/2/20.
//

#ifndef RTT_STPINFO_H
#define RTT_STPINFO_H

#include <roboteam_utils/Circle.h>
#include <roboteam_utils/Rectangle.h>
#include <world/Field.h>

#include "world_new/views/BallView.hpp"
#include "world_new/views/RobotView.hpp"

namespace rtt::ai::stp {

/**
 * Enum class used for status updates
 */
enum class Status {
    /**
     * Waiting for something
     */
    Waiting,
    /**
     * Skill / tactic has finalized running
     */
    Success,
    /**
     * Skill has failed
     */
    Failure,
    /**
     * Skill is executing
     */
    Running
};
/**
 * Enum class used for status updates
 */
static std::ostream& operator<<(std::ostream& os, Status status) {
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

/**
 * Class used for the Areas to avoid in TacticInfo
 * @tparam T Type of the shape for the areas to avoid
 */
template <typename T>
struct Areas {
    std::vector<T> areasToAvoid;

    /**
     * Checks whether the Robot is or would be in any of the shapes
     * @param pos Center of the robot
     * @return any(robot in shape for shape in areas)
     */
    [[nodiscard]] bool isInAny(Vector2 const& pos) const noexcept {
        Circle circle{pos, stp::control_constants::ROBOT_RADIUS};
        return std::any_of(areasToAvoid.begin(), areasToAvoid.end(), [&](auto const& area) { return area.intersects(circle); });
    }
};

const int blockLength = 3; // The number of elements in the blockdistance enum
enum BlockDistance { CLOSE = 1, HALFWAY, FAR }; // If you change this be sure to change blocklength also
enum KickChipType { PASS, TARGET, MAX };

struct StpInfo {
   public:
    const std::optional<world_new::view::RobotView>& getRobot() const { return robot; }
    void setRobot(const std::optional<world_new::view::RobotView>& robot) { this->robot = robot; }

    const std::optional<world_new::view::RobotView>& getEnemyRobot() const { return enemyRobot; }
    void setEnemyRobot(const std::optional<world_new::view::RobotView>& enemyRobot) { this->enemyRobot = enemyRobot; }

    const std::optional<world::Field>& getField() const { return field; }
    void setField(const std::optional<world::Field>& field) { this->field = field; }

    const std::optional<world_new::view::BallView>& getBall() const { return ball; }
    void setBall(const std::optional<world_new::view::BallView>& ball) { this->ball = ball; }

    const std::optional<Vector2>& getPositionToMoveTo() const { return positionToMoveTo; }
    void setPositionToMoveTo(const std::optional<Vector2>& position) { this->positionToMoveTo = position; }

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

    KickChipType getKickChipType() const { return kickChipType; }
    void setKickChipType(KickChipType kickChipType) { this->kickChipType = kickChipType; }

   private:
    /**
     * Robot this tactic applies to
     */
    std::optional<world_new::view::RobotView> robot;

    /**
     * EnemyRobot this tactic applies to
     */
    std::optional<world_new::view::RobotView> enemyRobot;

    /**
     * Field
     */
    std::optional<world::Field> field;

    /**
     * View to the ball in the world this tactic executes on
     */
    std::optional<world_new::view::BallView> ball;

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
    KickChipType kickChipType{};

    /**
     * Reference angle of the robot
     */
    Angle angle = Angle(0.0);

    /**
     * Speed of the dribbler in %
     */
    int dribblerSpeed = 0;

    /**
     * When blocking off a position, the robot is on line between a targetposition to block, and the enemy robot.
     * Used to decide how close this robot should be to enemy robot
     */
    BlockDistance blockDistance;
};
};  // namespace rtt::ai::stp

#endif  // RTT_STPINFO_H
