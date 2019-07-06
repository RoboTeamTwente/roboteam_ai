//
// Created by robzelluf on 6/14/19.
//

#include "Condition.h"

#ifndef ROBOTEAM_AI_CANREFLECTKICK_H
#define ROBOTEAM_AI_CANREFLECTKICK_H

namespace rtt {
namespace ai {

class CanReflectKick : public Condition {
private:
    const double MAX_BALL_RECEIVE_ANGLE = 70.0 / 180.0 * M_PI; // 70 degrees
    double getApproximateReflectAngle();
public:
    explicit CanReflectKick(std::string name = "CanReflectKick", bt::Blackboard::Ptr blackboard = nullptr);
    Status onUpdate() override;
};

}
}

#endif //ROBOTEAM_AI_CANREFLECTKICK_H
