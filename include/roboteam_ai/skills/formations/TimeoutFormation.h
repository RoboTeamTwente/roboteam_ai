//
// Created by mrlukasbos on 12-4-19.
//

#ifndef ROBOTEAM_AI_TIMEOUTFORMATION_H
#define ROBOTEAM_AI_TIMEOUTFORMATION_H

#include "Formation.h"

namespace rtt::ai {

class TimeoutFormation : public Formation {
   public:
    explicit TimeoutFormation(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

   private:
    Vector2 getFormationPosition() override;
    std::vector<world_new::view::RobotView> robotsInFormationPtr() override;
    static std::vector<world_new::view::RobotView> robotsInFormation;
};

}  // namespace rtt::ai
#endif  // ROBOTEAM_AI_TIMEOUTFORMATION_H
