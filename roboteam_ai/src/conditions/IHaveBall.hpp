//
// Created by rolf on 19-10-18.
//

#ifndef ROBOTEAM_AI_IHAVEBALL_HPP
#define ROBOTEAM_AI_IHAVEBALL_HPP

#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/WorldBall.h"
#include "Condition.h"
#include "../utilities/World.h"

namespace rtt{
namespace ai{
    bool bot_has_ball(boost::optional<const roboteam_msgs::WorldRobot>& bot, const roboteam_msgs::WorldBall& ball);
    /**
     * @class IHaveBall
     * @brief
     */
class IHaveBall : public Condition {
public:
    IHaveBall(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    /**
     * @brief checks if a robot with specific ID given through blackboards has a ball. Uses BB parameters "me/ROBOT_ID" and "our_team"
     * @return Returns status::success if a robot has a ball, status::failure otherwise.
     */
    Status Update() override;
    std::string node_name() override { return "IHaveBall";}

};
}
}

#endif //ROBOTEAM_AI_IHAVEBALL_HPP
