//
// Created by robzelluf on 11/13/18.
//

#include "../Tactic.h"

#ifndef ROBOTEAM_AI_PARALLELSEQUENCETEST_H
#define ROBOTEAM_AI_PARALLELSEQUENCETEST_H

namespace bt {

class ParallelSequenceTactic : public Tactic {

public:
    ParallelSequenceTactic(std::string name, Blackboard::Ptr blackboard);
    std::string name;
    void setName(std::string newName);
    void Initialize();
    Node::Status Update();
    std::string node_name() override;
    bool claimedRobots = false;
    std::set<int> robotIDs = {};
};

} //bt


#endif //ROBOTEAM_AI_PARALLELSEQUENCETEST_H
