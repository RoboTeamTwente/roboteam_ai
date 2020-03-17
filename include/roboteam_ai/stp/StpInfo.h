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
        Circle circle{pos, Constants::ROBOT_RADIUS()};
        return std::any_of(areasToAvoid.begin(), areasToAvoid.end(), [&](auto const& area) { return area.intersects(circle); });
    }
};


enum PositionType {
    MOVE_TO_POSITION,
    RECEIVE_AT_POSITION,
    SHOOT_TO_POSITION,
    DEFEND_POSITION
};
enum BlockDistance {
    CLOSE = 1,
    HALFWAY,
    FAR
};

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

    const std::pair<PositionType, Vector2>& getPosition() const { return position; }
    void setPosition(const std::pair<PositionType, Vector2>& position) { this->position = position; }

    double getKickChipVelocity() const { return kickChipVelocity; }
    void setKickChipVelocity(double kickChipVelocity) { this->kickChipVelocity = kickChipVelocity; }

    Angle getAngle() const { return angle; }
    void setAngle(double angle) { this->angle = Angle(angle); }

    int getDribblerSpeed() const { return dribblerSpeed; }
    void setDribblerSpeed(int dribblerSpeed) { this->dribblerSpeed = dribblerSpeed; }

    BlockDistance getBlockDistance() const { return blockDistance; }
    void setBlockDistance(BlockDistance blockDistance) { StpInfo::blockDistance = blockDistance; }

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

     * Tuple of the PositionType and the position of this target
     */
    std::pair<PositionType, Vector2> position;

    /**
     * Velocity of the kick/chip
     */
    double kickChipVelocity = 0.0;

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
