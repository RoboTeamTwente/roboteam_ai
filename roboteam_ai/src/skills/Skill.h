#ifndef ROBOTEAM_AI_SKILL_H
#define ROBOTEAM_AI_SKILL_H

#include "../bt/Leaf.hpp"
#include "ros/ros.h"
#include "../io/IOManager.h"
#include "roboteam_msgs/WorldRobot.h"
#include <roboteam_msgs/RobotCommand.h>
#include "../control/ControlUtils.h"
#include "../utilities/Constants.h"
#include "../utilities/Coach.h"
#include "roboteam_utils/Vector2.h"
#include <roboteam_ai/src/conditions/HasBall.hpp>
#include <roboteam_ai/src/utilities/Coach.h>
#include <roboteam_ai/src/control/ControlGoToPos.h>
#include "../control/PID.h"
#include "roboteam_utils/Arc.h"
#include "roboteam_utils/Math.h"

namespace rtt {
namespace ai {
namespace c = rtt::ai::constants;

/**
 * \class Skill
 * \brief Base class for all skills. Provides no additional functionality.
 */
class Skill : public bt::Leaf {
    protected:
        io::IOManager ioManager;

        using coach = coach::Coach;
        using goToType = control::ControlGoToPos::GoToType;
        void publishRobotCommand(roboteam_msgs::RobotCommand cmd);
    public:

        using Control = control::ControlUtils;
        using Status = bt::Node::Status;
        explicit Skill(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
        std::string node_name() override;
        void initialize() override;
        Status update() override;
        void terminate(Status s) override;

        virtual void onInitialize() { };
        virtual Status onUpdate() = 0;
        virtual void onTerminate(Status s) { };
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_SKILL_H
