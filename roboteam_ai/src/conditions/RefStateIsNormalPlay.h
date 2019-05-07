//
// Created by mrlukasbos on 2-5-19.
//

#ifndef ROBOTEAM_AI_REFSTATEISNORMALPLAY_H
#define ROBOTEAM_AI_REFSTATEISNORMALPLAY_H


#include "Condition.h"

namespace rtt{
namespace ai{

class RefStateIsNormalPlay : public Condition {
public:
    explicit RefStateIsNormalPlay(std::string name = "RefStateIsNormalPlay", bt::Blackboard::Ptr blackboard = nullptr);
    Status onUpdate() override;
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_REFSTATEISNORMALPLAY_H
