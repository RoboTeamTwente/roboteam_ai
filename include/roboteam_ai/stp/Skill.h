//
// Created by john on 3/2/20.
//

#ifndef RTT_SKILL_H
#define RTT_SKILL_H

#include <networking/RobotCommand.pb.h>

#include "stp/StpInfo.h"
#include "world/views/RobotView.hpp"

namespace rtt::ai::stp {

/**
 * Base skill class, inherit from it for making your own skill
 */
class Skill {
   protected:
    /**
     * Status of the last time update() was called
     */
    Status currentStatus;

    /**
     * Robot command that will eventually be sent to the robot
     */
    proto::RobotCommand command;

    /**
     * Robot this skill controls
     */
    std::optional<world::view::RobotView> robot;

    /**
     * Publishes the current robot command, limits it and refreshes it
     */
    virtual void publishRobotCommand(world::World const* data) noexcept;

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
     * Function that's called when the skill gets updated (every tick)
     * @param info StpInfo structure that provides data to the skill
     * @return Status according to its current execution
     */
    virtual Status onUpdate(StpInfo const& info) noexcept = 0;

   public:
    /**
     * Gets the status from the last time update() was called
     * @return this->currentStatus
     */
    [[nodiscard]] Status getStatus() const;

    /**
     * Calls onInitialize
     * @return Status of initialization
     */
    virtual void initialize() noexcept;

    /**
     * Function that's called when the skill gets updated (every tick)
     * @param info StpInfo structure that provides data to the skill
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

    /**
     * Gets the current skill name
     */
    virtual const char* getName() = 0;
};
}  // namespace rtt::ai::stp

#endif  // RTT_SKILL_H
