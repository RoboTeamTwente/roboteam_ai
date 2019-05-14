//
// Created by robzelluf on 5/14/19.
//

#ifndef ROBOTEAM_AI_ISBALLMOVINGFAST_H
#define ROBOTEAM_AI_ISBALLMOVINGFAST_H

#include "Condition.h"

namespace rtt {
namespace ai {

class IsBallMovingFast : public Condition {
private:
    double minVelocity = Constants::BALL_MOVING_FAST_VEL();
public:
    explicit IsBallMovingFast(std::string name = "IsBallMovingFast", bt::Blackboard::Ptr blackboard = nullptr);
    void onInitialize() override;
    Status onUpdate() override;

};

}
}


#endif //ROBOTEAM_AI_ISBALLMOVINGFAST_H
