//
// Created by john on 3/2/20.
//

#ifndef RTT_STPINFO_H
#define RTT_STPINFO_H

#include <roboteam_utils/Circle.h>
#include <roboteam_utils/Rectangle.h>

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
    [[nodiscard]] bool isInAny(Vector2 const &pos) const noexcept {
        Circle circle{pos, Constants::ROBOT_RADIUS()};
        return std::any_of(areasToAvoid.begin(), areasToAvoid.end(), [&](auto const &area) { return area.intersects(circle); });
    }
};

/**
 * Structure that represents the info passed to tactics
 */
class TacticInfo {
   public:
    [[nodiscard]] const world_new::view::BallView &getBall() const { return ball; }

    void setBall(const world_new::view::BallView &ball) { TacticInfo::ball = ball; }

    [[nodiscard]] const world_new::view::RobotView &getPassRobot() const { return passRobot; }

    void setPassRobot(const world_new::view::RobotView &passRobot) { TacticInfo::passRobot = passRobot; }

    [[nodiscard]] const Vector2 &getPosition() const { return position; }

    void setPosition(const Vector2 &position) { TacticInfo::position = position; }

    [[nodiscard]] float getMaxSpeed() const { return maxSpeed; }

    void setMaxSpeed(float maxSpeed) { TacticInfo::maxSpeed = maxSpeed; }

    [[nodiscard]] const Areas<Rectangle> &getAreasToAvoid() const { return areasToAvoid; }

    void setAreasToAvoid(const Areas<Rectangle> &areasToAvoid) { TacticInfo::areasToAvoid = areasToAvoid; }

   private:
    /**
     * View to the ball in the world this tactic executes on
     */
    world_new::view::BallView ball;
    /**
     * Robot to pass to
     */
    world_new::view::RobotView passRobot;

    /**
     * Target position for tactic
     */
    Vector2 position;

    /**
     * Maximum speed for the robot
     */
    float maxSpeed;

    /**
     * Areas to avoid, read Areas<T> for more info
     */
    Areas<Rectangle> areasToAvoid;

    TacticInfo(world_new::view::BallView ball, world_new::view::RobotView passRobot, world_new::view::WorldDataView world, Vector2 position, float maxspeed)
        : ball{ball}, passRobot{passRobot}, position{position}, maxSpeed{maxSpeed} {}
};

/**
 * Skill info used in skills
 */
class SkillInfo {
   public:
    [[nodiscard]] const TacticInfo &getTacticInfo() const { return tacticInfo; }

    void setTacticInfo(const TacticInfo &tacticInfo) {
        this->tacticInfo = tacticInfo;
    }

    [[nodiscard]] const world_new::view::RobotView &getRobot() const { return robot; }

    void setRobot(const world_new::view::RobotView &robot) {
        this->robot = robot;
    }

   private:
    /**
     * TacticInfo of the tactic that called the skill
     */
    TacticInfo tacticInfo;

    /**
     * Robot this skill applies to
     */
    world_new::view::RobotView robot;
};
};  // namespace rtt::ai::stp

#endif  // RTT_STPINFO_H
