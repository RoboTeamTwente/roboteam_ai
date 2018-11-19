//
// Created by baris on 24/10/18.
//

#ifndef ROBOTEAM_AI_GOTOPOS_H
#define ROBOTEAM_AI_GOTOPOS_H

#include "Skill.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {
namespace ai {

class GoToPos : public Skill {
    private:
        using status = bt::Node::Status;

        bool goToBall;
        bool goBehindBall;
        roboteam_msgs::WorldRobot robot;

        enum Progression {
          ON_THE_WAY, DONE, FAIL, INVALID
        };
        Progression currentProgress;
        Progression checkProgression();

        Vector2 deltaPos;
        Vector2 targetPos;

        bool checkTargetPos(Vector2 pos);

        void sendMoveCommand();

        bool commandSend;

    public:
        explicit GoToPos(string name, bt::Blackboard::Ptr blackboard);

        void Initialize() override;
        Status Update() override;
        void Terminate(status s) override;

        std::string node_name() override;

};
} // ai
} // rtt

#endif //ROBOTEAM_AI_GOTOPOS_H
