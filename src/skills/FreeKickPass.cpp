//
// Created by robzelluf on 7/2/19.
//

#include <skills/FreeKickPass.h>

namespace rtt::ai {

FreeKickPass::FreeKickPass(std::string name, bt::Blackboard::Ptr blackboard) : Pass(std::move(name), std::move(blackboard)) {}

void FreeKickPass::onInitialize() {
    if (properties->hasInt("maxTries")) {
        maxTries = properties->getInt("maxTries");
    } else {
        maxTries = 3;
    }

    robotToPassToID = -1;
    passInitialized = false;
    hasShot = false;
    fails = 0;
    forcePass = false;
}

void FreeKickPass::makeCommand() {
    auto shotData = robot->getControllers().getShotController()->getRobotCommand(*field, *robot, getKicker(), false,
                                                                                 control::PASS, control::LOW,
                                                                                 ball.value(), *world);
    command = shotData.makeROSCommand();
}

}  // namespace rtt::ai
