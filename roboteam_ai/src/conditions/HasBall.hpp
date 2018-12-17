//
// Created by rolf on 19-10-18.
//

#ifndef ROBOTEAM_AI_IHAVEBALL_HPP
#define ROBOTEAM_AI_IHAVEBALL_HPP

#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/WorldBall.h"
#include "Condition.h"
#include "../utilities/World.h"

namespace rtt {
namespace ai {

class HasBall : public Condition {

    private:

        using Status = bt::Node::Status;

        bool botHasBall(Vector2 ballPos);

    public:

        explicit HasBall(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

        /**
         * @brief checks if a robot with specific ID given through blackboards has a ball.
         * @return Returns status::success if a robot has a ball, status::failure otherwise.
         */
        Status update() override;
        std::string node_name() override { return "HasBall"; }

};

}
}

#endif //ROBOTEAM_AI_IHAVEBALL_HPP
