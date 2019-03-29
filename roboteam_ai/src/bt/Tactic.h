//
// Created by baris on 06/11/18.
//

#ifndef ROBOTEAM_AI_TACTIC_H
#define ROBOTEAM_AI_TACTIC_H

#include "Node.hpp"

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
        using robotType = robotDealer::RobotType;
        void initialize() override;
        Status update() override;
        void askForRobots();
        void terminate(Status s) override;
        std::vector<Node::Ptr> children;
        std::string node_name() override;
        std::string name;
};
}

#endif //ROBOTEAM_AI_TACTIC_H
