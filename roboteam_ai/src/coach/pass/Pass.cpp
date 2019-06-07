//
// Created by mrlukasbos on 7-6-19.
//

#include <roboteam_ai/src/world/World.h>
#include "Pass.h"

namespace rtt {
namespace ai {

// constuctors
Pass::Pass(std::shared_ptr<world::Robot> passer, std::shared_ptr<world::Robot> receiver)
        : passer(passer), receiver(receiver) { }

Pass::Pass(std::shared_ptr<world::Robot> passer, std::shared_ptr<world::Robot> receiver, Vector2 endPos,
           bool ballShouldLayStill)
        : passer(passer), receiver(receiver), passEnd(endPos), ballShouldLayStill(ballShouldLayStill) { }

Line Pass::getPassLine() {
    return {world::world->getBall()->pos, receiver->pos};
}

bool Pass::isPassKicked() const {
    return passKicked;
}

void Pass::setPassKicked(bool kicked) {
    if (!passKicked) {
        passKicked = kicked;
        passKickedTime = ros::Time::now();
    }
}

bool Pass::isBallShouldLayStill() const {
    return ballShouldLayStill;
}

void Pass::setBallShouldLayStill(bool shouldLayStill) {
    this->ballShouldLayStill = shouldLayStill;
}

}
}

