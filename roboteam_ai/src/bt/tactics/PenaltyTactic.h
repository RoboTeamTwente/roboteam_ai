//
// Created by baris on 18-3-19.
//

#ifndef ROBOTEAM_AI_PENALTYTACTIC_H
#define ROBOTEAM_AI_PENALTYTACTIC_H

#include <roboteam_ai/src/utilities/RobotDealer.h>
#include "../Tactic.h"

namespace bt {

class PenaltyTactic : public Tactic {
    public:
        using dealer = robotDealer::RobotDealer;
        std::map<std::string, robotType> robots;
        PenaltyTactic(std::string name, Blackboard::Ptr blackboard);

        void initialize() override;
};
} // bt

#endif //ROBOTEAM_AI_PENALTYTACTIC_H
