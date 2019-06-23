//
// Created by rolf on 17-6-19.
//

#ifndef ROBOTEAM_AI_KICKTO_H
#define ROBOTEAM_AI_KICKTO_H
#include "Skill.h"
#include <roboteam_ai/src/control/PosController.h>
#include <roboteam_ai/src/control/shotControllers/ShotController.h>

namespace rtt {
namespace ai {

class KickTo : public Skill {
    private:
        Vector2 shootPos={0.0,0.0};
    public:
        explicit KickTo(string name, bt::Blackboard::Ptr blackboard);
        Status onUpdate() override;
        void onInitialize() override;

};

} // ai
} // rtt


#endif //ROBOTEAM_AI_KICKTO_H
