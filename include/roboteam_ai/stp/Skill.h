//
// Created by john on 3/2/20.
//

#ifndef RTT_SKILL_H
#define RTT_SKILL_H

#include <bt/Node.h>
#include <world_new/views/RobotView.hpp>
#include <roboteam_proto/RobotCommand.pb.h>

#include "stp/StpInfo.h"

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
         * Terminates the skill
         * @return Status of termination
         */
        virtual Status onTerminate() noexcept = 0;

        /**
         * Function that's called when the skill gets updated (every tick)
         * @param info SkillInfo structure that provides data to the skill
         * @return Status according to its current execution
         */
        virtual Status onUpdate(SkillInfo const& info) noexcept = 0;

        /**
         * Initializes the skill
         * @return Status of initialization
         */
        virtual Status onInitialize() noexcept = 0;

        /**
         * Resets all the robot controllers in the RobotController struct
         */
        void refreshRobotPositionControllers() const noexcept;

    public:
        /**
         * Calls onInitialize
         * @return Status of initialization
         */
        virtual Status initialize() noexcept;

        /**
         * Function that's called when the skill gets updated (every tick)
         * @param info SkillInfo structure that provides data to the skill
         * @return Status according to its current execution
         */
        virtual Status update(SkillInfo const& info) noexcept;

        /**
         * Calls onTerminate
         * @return Status of termination
         */
        virtual Status terminate() noexcept;

        /**
         * Virtual dtor that ensures proper destruction
         */
        virtual ~Skill() = default;
    };
}

#endif //RTT_SKILL_H
