//
// Created by jaro on 15-10-20.
//

#ifndef RTT_CONTROLMODULE_H
#define RTT_CONTROLMODULE_H

#include <roboteam_proto/RobotCommand.pb.h>

#include "world/views/RobotView.hpp"

namespace rtt::ai::control {

/**
 * Control module as seen in the system architecture.
 * Its job: Receive RobotCommands from Skills, check these and limit and/or change things wherever necessary.
 * For now, it limits the velocity and acceleration that a Skill sends so the robot doesn't do a backflip.
 * And it checks if the command that was sent makes sense (e.g., no kicking and chipping at the same time.
 */
    class ControlModule {
    private:
        /**
         * Robot command that will eventually be sent to the robot
         */
        proto::RobotCommand command;

        /**
         * Robot this skill controls
         */
        std::optional<world::view::RobotView> robot;

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

    };
}  // namespace rtt::ai::control

#endif //RTT_CONTROLMODULE_H
