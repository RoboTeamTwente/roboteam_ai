//
// Created by jaro on 15-10-20.
//

#ifndef RTT_CONTROLMODULE_H
#define RTT_CONTROLMODULE_H

#include <roboteam_proto/RobotCommand.pb.h>

#include "stp/StpInfo.h"
#include "world/views/RobotView.hpp"

namespace rtt::ai::control {

/**
 * @author Jaro Kuiken
 * @brief Control Module as seen in the system architecture.
 * Its job: Receive RobotCommands from Skills, check these and limit and/or change things wherever necessary.
 * For now, it limits the velocity and acceleration that a Skill sends so the robot doesn't do a backflip.
 * And it checks if the command that was sent makes sense (e.g., no kicking and chipping at the same time.
 */
    class ControlModule {
    protected:
        /**
         * Robot command that will eventually be sent to the robot
         */
        static proto::RobotCommand command;

        /**
         * Robot forwarded by the skill
         */
        static std::optional<world::view::RobotView> robot;

        /**
         * Applies constraints to the internal robot command
         */
        static void limitRobotCommand() noexcept;

        /**
         * Limits the velocity with a control_constants value
         */
        static void limitVel() noexcept;

        /**
         * Limits the angular velocity with a control_constants value
         */
        static void limitAngularVel() noexcept;

        /**
         * Rotates the robot command to the other side of the field
         */
        static void rotateRobotCommand() noexcept;

    public:
        /**
         * Limits the current robot command and publishes it
         */
        static void publishRobotCommand(std::optional<world::view::RobotView> robot, const proto::RobotCommand& command, world::World const* data) noexcept;
    };
}  // namespace rtt::ai::control

#endif //RTT_CONTROLMODULE_H
