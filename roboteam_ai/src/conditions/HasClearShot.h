//
// Created by robzelluf on 3/14/19.
//

#ifndef ROBOTEAM_AI_SHOTATGOAL_H
#define ROBOTEAM_AI_SHOTATGOAL_H

#include "Condition.h"
#include "../utilities/Coach.h"

namespace rtt {
namespace ai {

class HasClearShot : public Condition {
public:
    explicit HasClearShot(std::string name = "HasClearShot", bt::Blackboard::Ptr blackboard = nullptr);
    void onInitialize() override;
    Status onUpdate() override;
    std::string node_name() override;

};

}
}


#endif //ROBOTEAM_AI_SHOTATGOAL_H
