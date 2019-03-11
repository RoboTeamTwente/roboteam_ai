//
// Created by baris on 11-3-19.
//

#include "ShootPenalty.h"
namespace rtt{
namespace ai {


void ShootPenalty::onInitialize() {

    // Check the shooting angle and geneva state
    // ask ROb
}



Skill::Status ShootPenalty::onUpdate() {

    // Shoot and then make sure that you dont double touch the ball
    // so go to a good position avoiding the ball



    return Status::Failure;
}



void ShootPenalty::onTerminate(Skill::Status s) {
    // clean up the coach or whereever logic you use

}
ShootPenalty::ShootPenalty(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}

}
}