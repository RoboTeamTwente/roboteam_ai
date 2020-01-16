//
// Created by rolf on 17-6-19.
//

#ifndef ROBOTEAM_AI_KICKTO_H
#define ROBOTEAM_AI_KICKTO_H
#include <control/PosController.h>
#include <control/shotControllers/ShotController.h>
#include "Skill.h"

namespace rtt::ai {

class KickTo : public Skill {
   private:
    Vector2 shootPos = {0.0, 0.0};

   public:
    explicit KickTo(string name, bt::Blackboard::Ptr blackboard);
    Status onUpdate() override;
    void onInitialize() override;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_KICKTO_H
