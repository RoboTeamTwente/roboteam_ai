//
// Created by rolf on 12-4-19.
//

#ifndef ROBOTEAM_AI_DEFENDERGETTINGBALL_H
#define ROBOTEAM_AI_DEFENDERGETTINGBALL_H

#include "Condition.h"
namespace rtt{
namespace ai{
class DefenderGettingBall : Condition{
    public:
        explicit DefenderGettingBall(std::string name = "DefenderGettingBall", bt::Blackboard::Ptr blackboard = nullptr);
        Status onUpdate() override;
        std::string node_name() override;
};
}
}


#endif //ROBOTEAM_AI_DEFENDERGETTINGBALL_H
