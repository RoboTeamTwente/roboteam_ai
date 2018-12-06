//
// Created by mrlukasbos on 6-12-18.
//

#ifndef ROBOTEAM_AI_HALTTACTIC_H
#define ROBOTEAM_AI_HALTTACTIC_H

#include "../Tactic.h"

namespace bt {

class HaltTactic : public Tactic {
public:
    HaltTactic(std::string name, Blackboard::Ptr blackboard);
    std::string name;
    void setName(std::string newName);
    void initialize() override;
    Node::Status update() override;
    void terminate(Status s) override;
    std::string node_name() override;
    int claimedRobots = 0;
    std::set<int> robotIDs = {};
};

} // bt

#endif //ROBOTEAM_AI_HALTTACTIC_H
