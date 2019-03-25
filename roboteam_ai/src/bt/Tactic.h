//
// Created by baris on 06/11/18.
//

#ifndef ROBOTEAM_AI_TACTIC_H
#define ROBOTEAM_AI_TACTIC_H

#include "Node.hpp"
#include "roboteam_ai/src/utilities/RobotDealer.h"

namespace robotDealer {
enum RobotType : short;
}

namespace bt {

class Tactic : public Node {
protected:
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
        Node::Ptr child = nullptr;
        std::string node_name() override;
        std::string name;
};
}

#endif //ROBOTEAM_AI_TACTIC_H
