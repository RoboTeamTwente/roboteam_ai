//
// Created by rolf on 04/12/18.
//

#ifndef ROBOTEAM_AI_GETBALL_H
#define ROBOTEAM_AI_GETBALL_H

#include "Skill.h"
#include "../control/positionControllers/BallHandlePosControl.h"

namespace rtt {
namespace ai {

class GetBall : public Skill {
        const double SPEED = (Constants::GRSIM() ? 0.4 : 0.8);
    private:
        control::BallHandlePosControl ballControlGtp = control::BallHandlePosControl(false);

    public:
        explicit GetBall(string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
        void onTerminate(Status s) override;

};
}
}

#endif //ROBOTEAM_AI_GETBALL_H
