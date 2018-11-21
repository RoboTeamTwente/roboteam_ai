//
// Created by baris on 05/11/18.
//
#include "../Tactic.h"

#ifndef ROBOTEAM_AI_TACTICNODE_H
#define ROBOTEAM_AI_TACTICNODE_H
namespace bt {

class DemoTactic : public Tactic {

    public:
        DemoTactic(std::string name, Blackboard::Ptr blackboard);

        std::string name;

        void setName(std::string newName);

        void initialize() override;
        Node::Status update() override;
        void terminate(Status s) override;

        std::string node_name() override;

        int claimedRobots = 0;

        std::set<int> robotIDs = {};

//        Node::Ptr child = nullptr;



};

} // bt

#endif //ROBOTEAM_AI_TACTICNODE_H
