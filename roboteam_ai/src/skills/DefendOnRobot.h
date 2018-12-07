//
// Created by robzelluf on 12/7/18.
//

#ifndef ROBOTEAM_AI_DEFENDONROBOT_H
#define ROBOTEAM_AI_DEFENDONROBOT_H


#include "Skill.h"
#include <boost/optional.hpp>
#include "../utilities/World.h"

namespace rtt {
namespace ai {

class DefendOnRobot : public Skill {
private:
    using status = bt::Node::Status;
    int amountOfCycles{};
protected:
    roboteam_msgs::WorldRobot robot;
    enum Progression {
        IDLE, DONE, FAIL
    };
    Progression currentProgress;
public:
    explicit DefendOnRobot(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);

    void initialize() override;

    Status update() override;

    Vector2 calculateBestPosition(roboteam_msgs::WorldRobot robot, roboteam_msgs::WorldRobot robot2);
};
} // ai
} // rtt


#endif //ROBOTEAM_AI_DEFENDONROBOT_H
