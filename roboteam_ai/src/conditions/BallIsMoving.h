//
// Created by mrlukasbos on 26-4-19.
//

#ifndef ROBOTEAM_AI_BALLISMOVING_H
#define ROBOTEAM_AI_BALLISMOVING_H

#include "Condition.h"

namespace rtt{
namespace ai{

class BallIsMoving : public Condition {
private:
    bool theirDefenceArea;
    bool outsideField = false;
public:
    explicit BallIsMoving(std::string name = "BallIsMoving", bt::Blackboard::Ptr blackboard = nullptr);
    Status onUpdate() override;
};

} // ai
} // rtt


#endif //ROBOTEAM_AI_BALLINDEFENSEAREAANDSTILL_H
