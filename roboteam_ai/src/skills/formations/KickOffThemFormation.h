//
// Created by mrlukasbos on 15-4-19.
//

#ifndef ROBOTEAM_AI_KICKOFFTHEMFORMATION_H
#define ROBOTEAM_AI_KICKOFFTHEMFORMATION_H

#include "roboteam_ai/src/skills/formations/Formation.h"

namespace rtt {
namespace ai {

class KickOffThemFormation : public Formation {
public:
    explicit KickOffThemFormation(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
private:
    Vector2 getFormationPosition() override;
};

}
}



#endif //ROBOTEAM_AI_KICKOFFFORMATION_H
