//
// Created by baris on 07/11/18.
//

#include "Node.hpp"
#ifndef ROBOTEAM_AI_ROLE_H
#define ROBOTEAM_AI_ROLE_H

namespace bt {

class Role : public Node {

        int ROBOT_ID = -1;

        Node::Ptr child = nullptr;

        std::string name;

        void Initialize() override;

        Status Update() override;

        void AddChild(Node::Ptr newChild) override;

};
}

#endif //ROBOTEAM_AI_ROLE_H
