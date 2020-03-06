#ifndef ROBOTEAM_AI_KICKOFFFORMATION_H
#define ROBOTEAM_AI_KICKOFFFORMATION_H

#include "Formation.h"

namespace rtt::ai {

class KickOffUsFormation : public Formation {
   public:
    explicit KickOffUsFormation(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

   private:
    Vector2 getFormationPosition() override;
    std::vector<world_new::view::RobotView> robotsInFormationPtr() override;
    static std::vector<world_new::view::RobotView> robotsInFormation;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_KICKOFFFORMATION_H
