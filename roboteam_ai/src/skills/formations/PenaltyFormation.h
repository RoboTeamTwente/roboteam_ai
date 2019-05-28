//
// Created by baris on 15-4-19.
//

#ifndef ROBOTEAM_AI_PENALTYFORMATION_H
#define ROBOTEAM_AI_PENALTYFORMATION_H
#include "Formation.h"

namespace rtt {
namespace ai {

class PenaltyFormation : public Formation {
    public:
        explicit PenaltyFormation(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    private:
        Vector2 getFormationPosition() override;
        std::shared_ptr<std::vector<std::shared_ptr<Robot>>> robotsInFormationPtr() override;
        static std::shared_ptr<std::vector<std::shared_ptr<Robot>>> robotsInFormation;
    };
}
}



#endif //ROBOTEAM_AI_PENALTYFORMATION_H
