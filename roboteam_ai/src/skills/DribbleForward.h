//
// Created by robzelluf on 5/13/19.
//

#ifndef ROBOTEAM_AI_DRIBBLEFORWARD_H
#define ROBOTEAM_AI_DRIBBLEFORWARD_H

#include "Skill.h"
#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>

namespace rtt {
namespace ai {

class DribbleForward : public Skill {
private:
    Vector2 initialBallPos;
    enum Progression {
        GETTING_BALL,
        DRIBBLING
    };

    Progression currentProgress;

    double dribbleDistance = 0.9;
    Vector2 targetPos;
    control::BasicPosControl basicGtp;
public:
    explicit DribbleForward(string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;
};

}
}


#endif //ROBOTEAM_AI_DRIBBLEFORWARD_H
