//
// Created by rolf on 10/12/18.
//

#ifndef ROBOTEAM_AI_KEEPER_H
#define ROBOTEAM_AI_KEEPER_H

#include <control/BasicPosControl.h>
#include "Skill.h"
#include "roboteam_utils/Arc.h"
#include "roboteam_utils/Mathematics.h"
#include <world/BallPossession.h>

namespace rtt::ai {
class Keeper : public Skill {

        const double KEEPER_POSDIF = 0.01;
        const double MIN_ATTACKER_DIST=0.3;

    private:
        Arc blockCircle;
        Vector2 computeBlockPoint(const Vector2& defendPos);
        Vector2 goalPos;
        double goalwidth;
        void setGoalPosWithAttacker(RobotPtr attacker);
        control::BasicPosControl posController;
        rtt::Arc createKeeperArc();
    public:
        explicit Keeper(string name, bt::Blackboard::Ptr blackboard);
        Status onUpdate() override;
        void onInitialize() override;
        void onTerminate(Status s) override;
};
}

#endif //ROBOTEAM_AI_KEEPER_H
