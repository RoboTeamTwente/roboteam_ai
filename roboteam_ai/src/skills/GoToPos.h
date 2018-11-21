//
// Created by baris on 24/10/18.
//

#ifndef ROBOTEAM_AI_GOTOPOS_H
#define ROBOTEAM_AI_GOTOPOS_H

#include "Skill.h"

namespace rtt {
namespace ai {

class GoToPos : public Skill {
    private:
        using status = bt::Node::Status;
        roboteam_msgs::WorldRobot robot;

        bool goToBall;
        bool goBehindBall;

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
        std::string node_name() override;

        void initialize() override;
        Status update() override;
        void terminate(status s) override;


};
} // ai
} // rtt

#endif //ROBOTEAM_AI_GOTOPOS_H
