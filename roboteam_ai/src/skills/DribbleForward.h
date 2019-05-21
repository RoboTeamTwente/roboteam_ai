//
// Created by robzelluf on 5/13/19.
//

#ifndef ROBOTEAM_AI_DRIBBLEFORWARD_H
#define ROBOTEAM_AI_DRIBBLEFORWARD_H

#include "Skill.h"
#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
#include <roboteam_ai/src/control/positionControllers/BallHandlePosControl.h>

namespace rtt {
namespace ai {

class DribbleForward : public Skill {
private:
    Vector2 initialBallPos;
    double dribbleDistance = 0.8;
    Vector2 targetPos;
    control::BasicPosControl basicGtp;
    control::BallHandlePosControl ballHandlePosControl;
public:
    explicit DribbleForward(string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;
};

}
}


#endif //ROBOTEAM_AI_DRIBBLEFORWARD_H
