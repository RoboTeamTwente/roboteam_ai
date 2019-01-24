//
// Created by mrlukasbos on 23-1-19.
//

#ifndef ROBOTEAM_AI_ENTERFORMATIONTACTIC_H
#define ROBOTEAM_AI_ENTERFORMATIONTACTIC_H

#include "../Tactic.h"

namespace bt {

class EnterFormationTactic: public Tactic {
public:
    std::string name;
    int claimedRobots = 0;
    std::set<int> robotIDs;
    std::map<std::string, robotType> robots;
    EnterFormationTactic(std::string name, Blackboard::Ptr blackboard);
    void setName(std::string newName);
    void initialize() override;
    Node::Status update() override;
    void terminate(Status s) override;
    std::string node_name() override;
};
} // bt
#endif //ROBOTEAM_AI_ENTERFORMATIONTACTIC_H
