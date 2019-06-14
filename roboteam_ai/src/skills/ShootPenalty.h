//
// Created by baris on 11-3-19.
//

#ifndef ROBOTEAM_AI_SHOOTPENALTY_H
#define ROBOTEAM_AI_SHOOTPENALTY_H

#include "Skill.h"

namespace rtt {
namespace ai {

class ShootPenalty : public Skill {
private:
    bool genevaSet = false;
    int genevaState;
    Vector2 aimPoint;
    int determineGenevaState();

    int tick = 0;
    int genevaChangeTicks = 30;
public:
    explicit ShootPenalty(string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;


};

}
}

#endif //ROBOTEAM_AI_SHOOTPENALTY_H
