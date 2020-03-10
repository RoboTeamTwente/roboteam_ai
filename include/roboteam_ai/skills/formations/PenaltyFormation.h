//
// Created by baris on 15-4-19.
//

#ifndef ROBOTEAM_AI_PENALTYFORMATION_H
#define ROBOTEAM_AI_PENALTYFORMATION_H

#include "Formation.h"

namespace rtt::ai {

class PenaltyFormation : public Formation {
   public:
    explicit PenaltyFormation(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

   private:
    Vector2 getFormationPosition() override;
    std::vector<world_new::view::RobotView> robotsInFormationPtr() override;
    static std::vector<world_new::view::RobotView> robotsInFormation;
};
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_PENALTYFORMATION_H
