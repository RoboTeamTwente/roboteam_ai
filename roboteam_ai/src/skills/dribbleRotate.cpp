//
// Created by rolf on 14/12/18.
//
// TODO: Test real robot rotation speeds.
// TODO: Make the robot automatically slow down/speed up if the ball is going to one end of the dribbler. Control?
#include "dribbleRotate.h"
namespace rtt {
namespace ai {
DribbleRotate::DribbleRotate(rtt::string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) { }
void DribbleRotate::checkProgression() { }
void DribbleRotate::onInitialize() {
    if (properties->hasDouble("Angle")){
        targetAngle=properties->getDouble("Angle");
    }

}
DribbleRotate::Status DribbleRotate::onUpdate() { }
void DribbleRotate::onTerminate(Status s) { }

}
}