//
// Created by rolf on 21/11/18.
//

#ifndef ROBOTEAM_AI_GRSIMTESTTACTIC_H
#define ROBOTEAM_AI_GRSIMTESTTACTIC_H
#include "../Tactic.h"
namespace bt {
class grsimTestTactic : public Tactic {

public:
grsimTestTactic(std::string name, Blackboard::Ptr blackboard);

std::string name;

void setName(std::string newName);

void initialize() override;
Node::Status update() override;

std::string node_name() override;

int claimedRobots = 0;

std::set<int> robotIDs;

};

}
#endif //ROBOTEAM_AI_GRSIMTESTTACTIC_H
