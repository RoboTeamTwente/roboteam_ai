//
// Created by baris on 23-4-19.
//

#ifndef ROBOTEAM_AI_FREEKICKFORMATION_H
#define ROBOTEAM_AI_FREEKICKFORMATION_H

#include "Formation.h"
namespace rtt {
namespace ai {

class FreeKickFormation : public Formation {
    public:
        explicit FreeKickFormation(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    private:
        Vector2 getFormationPosition() override;
        std::shared_ptr<vector<std::shared_ptr<Robot>>> robotsInFormationPtr() override;
        static std::shared_ptr<vector<std::shared_ptr<Robot>>> robotsInFormation;
};
}
}
#endif //ROBOTEAM_AI_FREEKICKFORMATION_H
