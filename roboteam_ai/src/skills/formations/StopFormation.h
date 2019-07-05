//
// Created by roboteam on 4/07/19.
//

#ifndef ROBOTEAM_AI_STOPFORMATION_H
#define ROBOTEAM_AI_STOPFORMATION_H


#include "roboteam_ai/src/skills/formations/Formation.h"

namespace rtt {
namespace ai {

class StopFormation : public Formation {
    public:
        explicit StopFormation(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    private:
        Vector2 getFormationPosition() override;
        std::shared_ptr<std::vector<std::shared_ptr<Robot>>> robotsInFormationPtr() override;
        static std::shared_ptr<std::vector<std::shared_ptr<Robot>>> robotsInFormation;
        void updateFormation() override;
        void setFinalAngle() override;
};




}
}
#endif //ROBOTEAM_AI_STOPFORMATION_H
