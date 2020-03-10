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

/**
 * Structure that represents the info passed to tactics
 */
struct TacticInfo {
    /**
     * View to the ball in the world this tactic executes on
     */
    std::optional<world_new::view::BallView> ball;

    /**
     * Robot to pass to
     */
    std::optional<world_new::view::RobotView> passRobot;

    Vector2 getPosition() const { return position; };
    void setPosition(Vector2 position) { this->position = position; };

    std::optional<world::Field> getField() const { return field; };
    void setField(world::Field field) { this->field = field; };

    std::optional<world_new::view::RobotView> getRobot() const { return _robot; }
    void setRobot(std::optional<world_new::view::RobotView> robot) { this->_robot = robot; }

    /**
     * Maximum speed that something can have, the ball, the robot... whatever the skill needs
     */
    float maxSpeed;

    /**
     * Areas to avoid, read Areas<T> for more info
     */
    Areas<Rectangle> areasToAvoid;

   private:
    /**
     * Position, could be position of robot, or whatever position the skill needs, maybe target position
     */
    Vector2 position;

    /**
     * Field
     */
    std::optional<world::Field> field;

    /**
     * Robot this tactic applies to
     */
    std::optional<world_new::view::RobotView> _robot;
};

/**
 * Skill info used in skills
 */
struct SkillInfo {
    TacticInfo getTacticInfo() const { return tacticInfo; };
    void setTacticInfo(TacticInfo tacticInfo) { this->tacticInfo = tacticInfo; };

    std::optional<world_new::view::RobotView> getRobot() const { return robot; };
    void setRobot(world_new::view::RobotView robot) { this->robot = robot; };

    double getKickChipVelocity() const { return kickChipVelocity; };
    void setKickChipVelocity(double kickChipVelocity) { this->kickChipVelocity = kickChipVelocity; };

    float getAngle() const { return angle; };
    void setAngle(double angle) { this->angle = angle; };

    int getDribblerSpeed() const { return dribblerSpeed; };
    void setDribblerSpeed(int dribblerSpeed) { this->dribblerSpeed = dribblerSpeed; };

   private:
    /**
     * TacticInfo of the tactic that called the skill
     */
    TacticInfo tacticInfo;

    /**
     * Robot this skill applies to
     */
    std::optional<world_new::view::RobotView> robot;

    /**
     * Velocity of the kick/chip
     */
    double kickChipVelocity = 0.0;

    /**
     * Reference angle of the robot
     */
    float angle = 0.0;

    /**
     * Speed of the dribbler
     */
    int dribblerSpeed = 0;
};
};  // namespace rtt::ai::stp

#endif  // RTT_STPINFO_H
