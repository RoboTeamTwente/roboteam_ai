//
// Created by jessevw on 21.02.20.
//

#ifndef RTT_NEWSKILL_H
#define RTT_NEWSKILL_H

#include <include/roboteam_ai/treeinterp/Leaf.h>

#ifndef ROBOTEAM_AI_SKILL_H
#define ROBOTEAM_AI_SKILL_H

#include <roboteam_utils/Angle.h>
#include "include/roboteam_ai/utilities/IOManager.h"
#include "roboteam_proto/RobotCommand.pb.h"
#include "treeinterp/Leaf.h"

namespace rtt::ai {

// forward declare control Utils
namespace control {
class ControlUtils;
}

namespace world {
class Robot;
class Ball;
class WorldData;
}  // namespace world

/**
 * \class Skill
 * \brief Base class for all skills. Provides no additional functionality.
 */
class NewSkill : public bt::Leaf {
   private:
    /**
     * used if we are playing on the other side of the field. By default, our side is left.
     * @param cmd the command to be rotated
     * @return the command, but as though we are playing on the other side of the field
     */
    proto::RobotCommand rotateRobotCommand(proto::RobotCommand &cmd);

   protected:
    /**
     * Gives the robotcommand to the global instance of IO, which then publishes the command
     */
    void publishRobotCommand();
    /**
     * Creates a new, empty RobotCommand and assigns it to "command"
     */
    void refreshRobotCommand();
    proto::RobotCommand command;

    /**
     * If any values in the robotcommand are beyond what the robot can handle, it caps these values and raises an error message
     */
    void limitRobotCommand();

   public:
    NewSkill(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    std::string node_name() override;
    void initialize() override;
    Status update() override;
    void terminate(Status s) override;
    virtual void onInitialize() = 0;
    virtual Status onUpdate() = 0;
    virtual void onTerminate(Status s) = 0;
    void refreshRobotPositionControllers();
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_SKILL_H

#endif  // RTT_NEWSKILL_H
