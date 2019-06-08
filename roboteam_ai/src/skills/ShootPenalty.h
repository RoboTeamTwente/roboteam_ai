//
// Created by baris on 11-3-19.
//

#ifndef ROBOTEAM_AI_SHOOTPENALTY_H
#define ROBOTEAM_AI_SHOOTPENALTY_H

#include <roboteam_ai/src/control/BasicPosControl.h>
#include "roboteam_ai/src/coach/OffensiveCoach.h"
#include <roboteam_ai/src/control/shotControllers/ShotController.h>
#include "Skill.h"
#include "../world/Field.h"
namespace rtt {
namespace ai {

class ShootPenalty : public Skill {
public:
    explicit ShootPenalty(string name, bt::Blackboard::Ptr blackboard);
    Status onUpdate() override;


};

}
}

#endif //ROBOTEAM_AI_SHOOTPENALTY_H
