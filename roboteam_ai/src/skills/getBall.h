//
// Created by rolf on 04/12/18.
//

#ifndef ROBOTEAM_AI_GETBALL_H
#define ROBOTEAM_AI_GETBALL_H

#include "Skill.h"
namespace rtt {
namespace ai {
class GetBall : public Skill {
    private:
        using status=bt::Node::Status;
        roboteam_msgs::WorldRobot robot;
        enum Progression {
          FARTOBALL, CLOSETOBALL, SUCCESS, FAIL
        };
        Progression currentProgress;
        Progression checkProgression();
    public:
        explicit GetBall(string name, bt::Blackboard::Ptr blackboard);
        std::string node_name() override;

        void initialize() override;
        status update() override;
        void terminate(status s) override;

};
}
}

#endif //ROBOTEAM_AI_GETBALL_H
