//
// Created by thijs on 17-12-18.
//

#ifndef ROBOTEAM_AI_ATTACK_H
#define ROBOTEAM_AI_ATTACK_H

#include <roboteam_ai/src/control/positionControllers/PosController.h>
<<<<<<< HEAD
#include <roboteam_ai/src/control/shotControllers/ShotController.h>
=======
#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
>>>>>>> origin/development
#include "Skill.h"

namespace rtt {
namespace ai {

class Attack : public Skill {
    private:
        const double BEHIND_BALL_CHECK = 0.4;
        const double BEHIND_BALL_TARGET = 0.2;
        Vector2 deltaPos;
        Vector2 targetPos;
        bool shot = false;
        std::shared_ptr<control::PosController> gtp;
        control::ShotController shotControl;

public:
        explicit Attack(string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
        void onTerminate(Status s) override;
};

} // ai
} // rtt
#endif //ROBOTEAM_AI_ATTACK_H
