//
// Created by thijs on 17-12-18.
//

#ifndef ROBOTEAM_AI_ATTACK_H
#define ROBOTEAM_AI_ATTACK_H

#include <roboteam_ai/src/control/PosController.h>
#include <roboteam_ai/src/control/shotControllers/ShotController.h>
#include "Skill.h"

namespace rtt {
namespace ai {

class Attack : public Skill {
private:
    int maxRethink = 3;
    int rethinks = 0;
    Vector2 aimPoint;
public:
    explicit Attack(string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;
};

} // ai
} // rtt
#endif //ROBOTEAM_AI_ATTACK_H
