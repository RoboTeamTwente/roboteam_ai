//
// Created by mrlukasbos on 15-4-19.
//

#ifndef ROBOTEAM_AI_KICKOFFTHEMFORMATION_H
#define ROBOTEAM_AI_KICKOFFTHEMFORMATION_H

#include "skills/formations/Formation.h"

namespace rtt::ai {

class KickOffThemFormation : public Formation {
   public:
    explicit KickOffThemFormation(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

   private:
    Vector2 getFormationPosition() override;
    std::vector<world_new::view::RobotView> robotsInFormationPtr() override;
    static std::vector<world_new::view::RobotView> robotsInFormation;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_KICKOFFFORMATION_H
