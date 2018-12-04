//
// Created by rolf on 04/12/18.
//

#ifndef ROBOTEAM_AI_GETBALL_H
#define ROBOTEAM_AI_GETBALL_H

#include "Skill.h"
#include "../utilities/Constants.h"
#include "../control/ControlUtils.h"

namespace rtt {
namespace ai {
class GetBall : public Skill {
    private:
        using status=bt::Node::Status;
        roboteam_msgs::WorldRobot robot;
        roboteam_msgs::WorldBall ball;

        enum Progression {
          TURNING, APPROACHING, DRIBBLING, SUCCESS, FAIL
        };
        bool robothasBall();
        void sendTurnCommand();
        void sendApproachCommand();
        void sendDribblingCommand();
        Progression currentProgress;
        Progression checkProgression();

        Vector2 deltaPos;
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
