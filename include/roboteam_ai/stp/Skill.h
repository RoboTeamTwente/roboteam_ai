//
// Created by john on 3/2/20.
//

#ifndef RTT_SKILL_H
#define RTT_SKILL_H

#include <bt/Node.h>
#include <world_new/views/RobotView.hpp>

#include "stp/StpInfo.h"

namespace rtt::ai::stp {
/**
 * Base skill class, inherit from it for making your own skill
 */
class Skill {
   private:
    Status status;

   public:
    Status getStatus() const;

   private:
    /**
     * Robot command that will be used for operations, such as publishing
     */
    proto::RobotCommand command;

    /**
     * Robot view, which should be set from the SkillInfo
     */
    world_new::view::RobotView robot;

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
     * Function that takes a status and calls terminate (the overridden function in the derived class)
     */
    virtual void onTerminate() noexcept;

    /**
     * Function that's called when the skill gets updated (every tick)
     * @param info SkillInfo structure that provides data to the skill
     * @return Status according to its current execution
     */
    virtual void onUpdate(SkillInfo const& info) noexcept;

    /**
     * Function that calls initialize (the overridden function in the derived class)
     */
    virtual void onInitialize() noexcept;

    /**
     * Resets all the robot controllers in the RobotController struct
     */
    void refreshRobotPositionControllers() const noexcept;

   public:
    /**
     * Function that's called immediately when the skill is initialized
     */
    virtual void initialize() noexcept = 0;

    /**
     * Function that's called when the skill gets updated (every tick)
     * @param info SkillInfo structure that provides data to the skill
     * @return Status according to its current execution
     */
    virtual Status update(SkillInfo const& info) noexcept = 0;

    /**
     * Function that's called in onTerminate,
     * called after update() returns Success or Failure
     */
    virtual void terminate() noexcept = 0;
};
}  // namespace rtt::ai::stp

#endif  // RTT_SKILL_H
