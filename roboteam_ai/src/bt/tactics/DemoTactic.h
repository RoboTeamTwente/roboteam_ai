//
// Created by baris on 05/11/18.
//
#include "../Tactic.h"
#include "../../../src/utilities/RobotDealer.h"

#ifndef ROBOTEAM_AI_TACTICNODE_H
#define ROBOTEAM_AI_TACTICNODE_H
namespace bt {

class DemoTactic : public Tactic {

    private:

        DemoTactic(std::string name, Blackboard::Ptr blackboard);

        std::string name;

        void setName(std::string newName);

        void Initialize();

        Node::Status Update();

        using RobotDealer = rtt::ai::RobotDealer;

        bool claimedRobots = false;

        std::set<int> robotIDs = {};

        Node::Ptr child = nullptr;

        void AddChild(bt::Node::Ptr newChild);

        bool askForRandomRobots(int numberOfRobots);


};

} // bt

#endif //ROBOTEAM_AI_TACTICNODE_H
