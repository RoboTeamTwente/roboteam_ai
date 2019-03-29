//
// Created by mrlukasbos on 23-1-19.
//

#ifndef ROBOTEAM_AI_ENTERFORMATIONTACTIC_H
#define ROBOTEAM_AI_ENTERFORMATIONTACTIC_H

#include "../Tactic.h"
#include "../../utilities/RobotDealer.h"

namespace bt {

class EnterFormationTactic : public Tactic {
    public:
        using dealer = robotDealer::RobotDealer;
        std::map<std::string, RobotType> robots;
        EnterFormationTactic(std::string name, Blackboard::Ptr blackboard);
        void initialize() override;
};
} // bt
#endif //ROBOTEAM_AI_ENTERFORMATIONTACTIC_H
