//
// Created by thijs on 15-11-18.
//
#include "../Tactic.h"

#ifndef ROBOTEAM_AI_VICTORYDANCETACTIC_H
#define ROBOTEAM_AI_VICTORYDANCETACTIC_H


namespace bt {

class VictoryDanceTactic : public Tactic {
    public:
        VictoryDanceTactic(std::string name, Blackboard::Ptr blackboard);
        void initialize() override;
};

} // bt
#endif //ROBOTEAM_AI_VICTORYDANCETACTIC_H
