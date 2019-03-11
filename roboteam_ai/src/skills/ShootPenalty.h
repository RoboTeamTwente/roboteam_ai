//
// Created by baris on 11-3-19.
//

#ifndef ROBOTEAM_AI_SHOOTPENALTY_H
#define ROBOTEAM_AI_SHOOTPENALTY_H

#include "Skill.h"
namespace rtt {
namespace ai {

class ShootPenalty : public Skill {

        Status onUpdate() override;
        void onInitialize() override;
        void onTerminate(Status s) override;

};

}
}

#endif //ROBOTEAM_AI_SHOOTPENALTY_H
