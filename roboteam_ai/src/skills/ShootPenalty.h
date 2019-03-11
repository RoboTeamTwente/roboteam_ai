//
// Created by baris on 11-3-19.
//

#ifndef ROBOTEAM_AI_SHOOTPENALTY_H
#define ROBOTEAM_AI_SHOOTPENALTY_H

#include "Skill.h"
namespace rtt {
namespace ai {

class ShootPenalty : public Skill {
    public:
        explicit ShootPenalty(string name, bt::Blackboard::Ptr blackboard);

    private:
        Status onUpdate() override;
        void onInitialize() override;
        void onTerminate(Status s) override;

};

}
}

#endif //ROBOTEAM_AI_SHOOTPENALTY_H
