//
// Created by baris on 06/11/18.
//

#ifndef ROBOTEAM_AI_TACTIC_H
#define ROBOTEAM_AI_TACTIC_H

#include "Node.hpp"
#include "roboteam_ai/src/utilities/RobotDealer.h"

namespace bt {

class Tactic : public Node {
protected:
        using dealer = rtt::ai::robotDealer::RobotDealer;
    int claimedRobots = 0;
    std::set<int> robotIDs;
public:
        void addChild(Node::Ptr newChild) override;

        std::vector<Node::Ptr> getChildren() override;
        using RobotType = rtt::ai::robotDealer::RobotType;
        void initialize() override;
        Status update() override;
        void askForRobots();
        void terminate(Status s) override;
        std::vector<Node::Ptr> children;
        std::string node_name() override;
        std::string name;
        void giveProperty(std::string a, std::string b) override;

};
}

#endif //ROBOTEAM_AI_TACTIC_H
