//
// Created by thijs on 19-11-18.
//

#ifndef ROBOTEAM_AI_DEFAULTSKILL_H
#define ROBOTEAM_AI_DEFAULTSKILL_H

#include "Skill.h"

namespace rtt {
namespace ai {

class DefaultSkill : public Skill {

    private:

        using Status = bt::Node::Status;
        roboteam_msgs::WorldRobot robot;

        bool variable1;

    public:

        explicit DefaultSkill(string name, bt::Blackboard::Ptr blackboard);
        std::string node_name() override;

        void initialize() override;
        Status update() override;
        void terminate(Status s) override;



};
} // ai
} // rtt

#endif //ROBOTEAM_AI_DEFAULTSKILL_H
