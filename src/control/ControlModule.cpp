//
// Created by jaro on 15-10-20.
//

#include "control/ControlModule.h"

namespace rtt::ai::control {


    void ControlModule::limitRobotCommand() noexcept {
        limitVel();
        limitAngularVel();
    }

    void ControlModule::limitVel() noexcept {

    }

    void ControlModule::limitAngularVel() noexcept {

    }
}  // namespace rtt::ai::stp