//
// Created by robzelluf on 3/14/19.
//

#ifndef ROBOTEAM_AI_SHOTATGOAL_H
#define ROBOTEAM_AI_SHOTATGOAL_H

#include "Condition.h"
#include "../utilities/Coach.h"

namespace rtt {
namespace ai {

class ShotAtGoal : public Condition {
public:
    explicit ShotAtGoal(std::string name = "ShotAtGoal", bt::Blackboard::Ptr blackboard = nullptr);
    void initialize() override;
    Status update() override;
    std::string node_name() override;

};

}
}


#endif //ROBOTEAM_AI_SHOTATGOAL_H
