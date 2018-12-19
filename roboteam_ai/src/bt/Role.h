//
// Created by baris on 07/11/18.
//

#include "Node.hpp"

#ifndef ROBOTEAM_AI_ROLE_H
#define ROBOTEAM_AI_ROLE_H

namespace bt {

class Role : public Node {

    public:
        Role(std::string name);

        Node::Ptr child = nullptr;

        std::string name = "Role";

        void initialize() override;

        Status update() override;

        void addChild(Node::Ptr newChild) override;

        std::vector<Node::Ptr> getChildren() override;

        std::string node_name() override;

};
}

#endif //ROBOTEAM_AI_ROLE_H
