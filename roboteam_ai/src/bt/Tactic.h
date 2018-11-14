//
// Created by baris on 06/11/18.
//

#ifndef ROBOTEAM_AI_TACTIC_H
#define ROBOTEAM_AI_TACTIC_H

#include "Node.hpp"
namespace bt {

class Tactic : public Node {
    public:
        void AddChild(Node::Ptr newChild) override;


        void Initialize() override;

        Status Update() override;

        void askForRobots();


        void Terminate(Status s) override;

        Node::Ptr child = nullptr;
        std::string node_name() override;

        std::string name = "Tactic";

        int numberOfRobots = 0;
};
}

#endif //ROBOTEAM_AI_TACTIC_H
