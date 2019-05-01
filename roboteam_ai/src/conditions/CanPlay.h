//
// Created by mrlukasbos on 26-4-19.
//

#ifndef ROBOTEAM_AI_CANPLAY_H
#define ROBOTEAM_AI_CANPLAY_H

#include "Condition.h"

namespace rtt{
namespace ai{

class CanPlay : public Condition {
private:
    bool theirDefenceArea;
    bool outsideField = false;
public:
    explicit CanPlay(std::string name = "CanPlay", bt::Blackboard::Ptr blackboard = nullptr);
    Status onUpdate() override;
};

} // ai
} // rtt


#endif //ROBOTEAM_AI_CANPLAY_H
