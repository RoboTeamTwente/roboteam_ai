//
// Created by roboteam on 15-4-19.
//

#ifndef ROBOTEAM_AI_PENALTYFORMATION_H
#define ROBOTEAM_AI_PENALTYFORMATION_H
#include "roboteam_ai/src/skills/formations/EnterFormation.h"

namespace rtt {
namespace ai {

class PenaltyFormation : public EnterFormation {
    public:
        explicit PenaltyFormation(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    private:
        Vector2 getFormationPosition() override;

};
}
}

#endif //ROBOTEAM_AI_PENALTYFORMATION_H
