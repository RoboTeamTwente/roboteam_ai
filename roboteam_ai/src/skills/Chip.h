//
// Created by mrlukasbos on 23-10-18.
//

#ifndef ROBOTEAM_AI_CHIP_H
#define ROBOTEAM_AI_CHIP_H

#include "Kick.h"

namespace rtt {
namespace ai {

class Chip : public Kick {
    public:
        explicit Chip(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
        void sendKickCommand(double kickVel) override;
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_CHIP_H
