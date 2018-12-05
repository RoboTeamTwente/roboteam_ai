//
// Created by baris on 5-12-18.
//

#ifndef ROBOTEAM_AI_PASS_H
#define ROBOTEAM_AI_PASS_H

#include "Skill.h"

namespace rtt {
namespace ai {

class Pass : public Skill {

    private:

        using Status = bt::Node::Status;
        roboteam_msgs::WorldRobot robot;
        bool defensive;
        int getRobotToPass();
        int robotToPass;
        bool sendPassCommand();

    public:

        explicit Pass(string name, bt::Blackboard::Ptr blackboard);
        std::string node_name() override;

        void initialize() override;
        Status update() override;
        void terminate(Status s) override;

};
} // ai
} // rtt


#endif //ROBOTEAM_AI_PASS_H
