//
// Created by rolf on 12-4-19.
//

#ifndef ROBOTEAM_AI_ISDEFENDERGETTINGBALL_H
#define ROBOTEAM_AI_ISDEFENDERGETTINGBALL_H

#include "Condition.h"
namespace rtt{
namespace ai{
class IsDefenderGettingBall : public Condition{
    public:
        explicit IsDefenderGettingBall(std::string name = "IsDefenderGettingBall", bt::Blackboard::Ptr blackboard = nullptr);
        Status onUpdate() override;
        std::string node_name() override;
};
}
}


#endif //ROBOTEAM_AI_ISDEFENDERGETTINGBALL_H
