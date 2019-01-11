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
    public:
        void addChild(Node::Ptr newChild) override;

        std::vector<Node::Ptr> getChildren() override;
        using robotType = robotDealer::RobotType;

        void initialize() override;

        Status update() override;

        void askForRobots();

        void terminate(Status s) override;

        Node::Ptr child = nullptr;
        std::string node_name() override;

        std::string name = "Tactic";
};
}

#endif //ROBOTEAM_AI_TACTIC_H
