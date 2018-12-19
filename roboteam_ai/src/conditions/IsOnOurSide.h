//
// Created by robzelluf on 10/25/18.
//

#ifndef ROBOTEAM_AI_ISONSIDE_H
#define ROBOTEAM_AI_ISONSIDE_H

#include "Condition.h"

namespace rtt {
namespace ai {

class IsOnOurSide : public ai::Condition {
    public:
        IsOnOurSide(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
        Status update() override;
        std::string node_name() override { return "IsOnOurSide"; }
};

}// ai
}// rtt


#endif //ROBOTEAM_AI_ISONSIDE_H
