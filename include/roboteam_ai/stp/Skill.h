//
// Created by john on 3/2/20.
//

#ifndef RTT_SKILL_H
#define RTT_SKILL_H

#include <bt/Node.h>
#include <roboteam_proto/RobotCommand.pb.h>
#include <world_new/views/RobotView.hpp>
#include "stp/StpInfo.h"
#include "stp/new_constants/ControlConstants.h"

namespace rtt::ai::stp {
/**
 * Base skill class, inherit from it for making your own skill
 */
class Skill {
   protected:
    /**
     * Robot command that will be used for operations, such as publishing
     */
    proto::RobotCommand command;

    /**
     * Robot view, which should be set from the SkillInfo
     */
    std::optional<world_new::view::RobotView> robot;

    /**
     * Gets the name of the skill, must be a literal
     * @return [{additional_info}] {class_name}
     */
    constexpr const char* name() const noexcept;

    /**
     * Publishes the current robot command, limits it and refreshes it
     */
    virtual void publishRobotCommand() noexcept;

    /**
     * Rotates the robot command to the other side of the field
     */
    virtual void rotateRobotCommand() noexcept;

    /**
     * Resets the internal robot command
     */
    virtual void refreshRobotCommand() noexcept;

    /**
     * Applies constraints to the internal robot command
     */
    virtual void limitRobotCommand() noexcept;

    /**
     * Limits the velocity
     */
    virtual void limitVel() noexcept;

    /**
     * Limits the angular velocity
     */
    virtual void limitAngularVel() noexcept;

    /**
     * Terminates the skill
     * @return Status of termination
     */
    virtual void onTerminate() noexcept = 0;

    /**
     * Function that's called when the skill gets updated (every tick)
     * @param info SkillInfo structure that provides data to the skill
     * @return Status according to its current execution
     */
    virtual Status onUpdate(StpInfo const& info) noexcept = 0;

    /**
     * Initializes the skill
     * @return Status of initialization
     */
    virtual void onInitialize() noexcept = 0;

    /**
     * Resets all the robot controllers in the RobotController struct
     */
    void refreshRobotPositionControllers() const noexcept;

   public:
    /**
     * Calls onInitialize
     * @return Status of initialization
     */
    virtual void initialize() noexcept;

    /**
     * Function that's called when the skill gets updated (every tick)
     * @param info SkillInfo structure that provides data to the skill
     * @return Status according to its current execution
     */
    virtual Status update(StpInfo const& info) noexcept;

    /**
     * Calls onTerminate
     * @return Status of termination
     */
    virtual void terminate() noexcept;

    /**
     * Virtual dtor that ensures proper destruction
     */
    virtual ~Skill() = default;
};
}  // namespace rtt::ai::stp

#endif  // RTT_SKILL_H
