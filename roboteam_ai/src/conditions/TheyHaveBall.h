//
// Created by robzelluf on 10/24/18.
//

#ifndef ROBOTEAM_AI_THEYHAVEBALL_H
#define ROBOTEAM_AI_THEYHAVEBALL_H

#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/WorldBall.h"
#include "Condition.h"
#include "../utilities/World.h"
#include "../utilities/Coach.h"

namespace rtt {
namespace ai {
/**
 * @class IHaveBall
 * @brief
 */
class TheyHaveBall : public Condition {
    public:
        TheyHaveBall(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
        /**
         * @brief checks if a robot with specific ID given through blackboards has a ball. Uses BB parameters "me/ROBOT_ID" and "our_team"
         * @return Returns status::success if a robot has a ball, status::failure otherwise.
         */
        Status update() override;
        std::string node_name() override { return "TheyHaveBall"; }

};
}
}

#endif //ROBOTEAM_AI_THEYHAVEBALL_H
