//
// Created by robzelluf on 11/13/18.
//

#include "../Tactic.h"

#ifndef ROBOTEAM_AI_PARALLELSEQUENCETEST_H
#define ROBOTEAM_AI_PARALLELSEQUENCETEST_H

namespace bt {

class ParallelSequenceTactic : public Tactic {
    private:

    public:
        ParallelSequenceTactic(std::string name, Blackboard::Ptr blackboard);
        std::string name;
        void setName(std::string newName);
        void Initialize() override;
        Node::Status Update() override;
        std::string node_name() override;
        int claimedRobots = 0;
        std::set<int> robotIDs;
};

} //bt


#endif //ROBOTEAM_AI_PARALLELSEQUENCETEST_H
