//
// Created by mrlukasbos on 12-4-19.
//

#ifndef ROBOTEAM_AI_TIMEOUTFORMATION_H
#define ROBOTEAM_AI_TIMEOUTFORMATION_H

#include "roboteam_ai/src/skills/formations/EnterFormation.h"

namespace rtt {
namespace ai {

class TimeoutFormation : public EnterFormation {
public:
    explicit TimeoutFormation(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
private:
    Vector2 getFormationPosition() override;
};




}
}
#endif //ROBOTEAM_AI_TIMEOUTFORMATION_H
