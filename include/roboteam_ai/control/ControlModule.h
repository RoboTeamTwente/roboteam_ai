//
// Created by jaro on 15-10-20.
//

#ifndef RTT_CONTROLMODULE_H
#define RTT_CONTROLMODULE_H

#include <mutex>
#include <roboteam_utils/RobotCommands.hpp>

#include "control/AnglePID.h"
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
     *
     */
    static inline std::mutex robotCommandsMutex;
    static inline std::vector<rtt::RobotCommand> robotCommands;
    static inline std::map<unsigned int, AnglePID> simulatorAnglePIDmap;

    /**
     * Applies constraints to the internal robot command
     */
    static void limitRobotCommand(rtt::RobotCommand& command, std::optional<rtt::world::view::RobotView> robot);

    /**
     * Limits the velocity with a control_constants value
     */
    static void limitVel(rtt::RobotCommand& command, std::optional<rtt::world::view::RobotView> robot);

    /**
     * Limits the angular velocity with a control_constants value
     */
    static void limitAngularVel(rtt::RobotCommand& command, std::optional<rtt::world::view::RobotView> robot);

    /**
     * Rotates the robot command to the other side of the field
     */
    static void rotateRobotCommand(rtt::RobotCommand& command);

   public:
    /**
     * Limits the current robot command and adds it to the list of commands to be sent
     */
    static void addRobotCommand(std::optional<rtt::world::view::RobotView> robot, const rtt::RobotCommand& command, const rtt::world::World* data) noexcept;
    /**
     *
     */
    static void sendAllCommands();

    static void simulator_angular_control(const std::optional<::rtt::world::view::RobotView>& robot, rtt::RobotCommand& robot_command);
};
}  // namespace rtt::ai::control

#endif  // RTT_CONTROLMODULE_H
