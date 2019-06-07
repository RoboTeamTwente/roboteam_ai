//
// Created by mrlukasbos on 7-6-19.
//

#include <roboteam_ai/src/world/World.h>
#include "Pass.h"

namespace rtt {
namespace ai {

// Pass between two robots, locations don't matter
Pass::Pass(std::shared_ptr<world::Robot> passer, std::shared_ptr<world::Robot> receiver)
        : passer(passer), receiver(receiver) {
    locationBasedPass = false;
    passInitializedTime = ros::Time::now();
}

// pass where the ball should be aimed at a specific location
Pass::Pass(std::shared_ptr<world::Robot> passer, std::shared_ptr<world::Robot> receiver, Vector2 endPos,
           bool ballShouldLayStill)
        : passer(passer), receiver(receiver), passEnd(endPos), ballShouldLayStill(ballShouldLayStill) {
    locationBasedPass = true;
    passInitializedTime = ros::Time::now();
}

// return the line of where the pass is expected to be
Line Pass::getPassLine() {
    return {world::world->getBall()->pos, getPassEnd()};
}

// return true if the pass has been kicked (this needs to be indicated by the passer)
bool Pass::isPassKicked() const {
    return passKicked;
}

// Indicate that the pass has been kicked. If it was already kicked then don't reset the time.
void Pass::setPassKicked(bool kicked) {
    if (!passKicked) {
        passKicked = kicked;
        passKickedTime = ros::Time::now();
    }
}

// return true if the ball should lay still at the end location
bool Pass::isBallShouldLayStill() const {
    return ballShouldLayStill;
}

void Pass::setBallShouldLayStill(bool shouldLayStill) {
    this->ballShouldLayStill = shouldLayStill;
}

// return true if the pass has been kicked but the ball is underway for too long
bool Pass::passTakesTooLong() {
    return passKicked && ros::Time::now().toSec() - passKickedTime.toSec() > MAX_PASS_TIME;
}

bool Pass::passFailed() {
    return passTakesTooLong();
}

bool Pass::passSucceeded() {
    return false;
}

const Vector2 &Pass::getPassEnd() const {
    if (locationBasedPass) return passEnd;
    return receiver->pos;
}

} // ai
} // rtt

