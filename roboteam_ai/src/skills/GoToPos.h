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

        bool goToBall;
        bool goBehindBall;

        enum Progression {
          ON_THE_WAY, DONE, FAIL
        };
        Progression currentProgress;
        Progression checkProgression();

        Vector2 deltaPos;
        Vector2 targetPos;

        bool checkTargetPos(Vector2 pos);

        void sendMoveCommand();
        void sendMoveCommand2();
        bool commandSend;

    public:
        explicit GoToPos(string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
        void onTerminate(status s) override;
};
} // ai
} // rtt

#endif //ROBOTEAM_AI_GOTOPOS_H
