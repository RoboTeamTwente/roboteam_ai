//
// Created by mrlukasbos on 12-4-19.
//

#ifndef ROBOTEAM_AI_TIMEOUTFORMATION_H
#define ROBOTEAM_AI_TIMEOUTFORMATION_H

#include "skills/formations/Formation.h"

namespace rtt::ai {

class TimeoutFormation : public Formation {
   public:
    explicit TimeoutFormation(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

   private:
    Vector2 getFormationPosition() override;
    std::shared_ptr<std::vector<std::shared_ptr<Robot>>> robotsInFormationPtr() override;
    static std::shared_ptr<std::vector<std::shared_ptr<Robot>>> robotsInFormation;
};

}  // namespace rtt::ai
#endif  // ROBOTEAM_AI_TIMEOUTFORMATION_H
