//
// Created by baris on 11-3-19.
//

#ifndef ROBOTEAM_AI_SHOOTPENALTY_H
#define ROBOTEAM_AI_SHOOTPENALTY_H

#include <include/roboteam_ai/control/BasicPosControl.h>
#include "include/roboteam_ai/coach/OffensiveCoach.h"
#include <include/roboteam_ai/control/shotControllers/ShotController.h>
#include "Skill.h"
#include "include/roboteam_ai/world/Field.h"
namespace rtt {
namespace ai {

class ShootPenalty : public Skill {
private:
    bool genevaSet = false;
    int genevaState;
    Vector2 aimPoint;
    int determineGenevaState();
    control::BasicPosControl gtp;
    int tick = 0;
    int genevaChangeTicks = 60;
    double lineP=0;
    Vector2 additionalBallDist;
    bool forcedKickOn=true;
    double forcedKickRange;

    Vector2 ballPos;
public:
    explicit ShootPenalty(string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;


};

}
}

#endif //ROBOTEAM_AI_SHOOTPENALTY_H
