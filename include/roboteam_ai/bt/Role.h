//
// Created by baris on 07/11/18.
//

#include "Node.hpp"

#ifndef ROBOTEAM_AI_ROLE_H
#define ROBOTEAM_AI_ROLE_H

namespace bt {

class Role : public Node {

    public:
    /**
     * By default, a role already has 1 child, initialised to nullptr. Therefore, calling setRole(string name) on a role without children will result in segfault
     * @param name the name you want the role to have. Be sure to call "setRole(string name) on this so all the nodes below it are also recursively given the same role name
     */
        Role(std::string name);

        Node::Ptr child = nullptr;

        std::string name = "Role";

        void initialize() override;

        Status update() override;

        void addChild(Node::Ptr newChild) override;

        std::vector<Node::Ptr> getChildren() override;

        std::string node_name() override;

        void terminate(Status s) override;


    };
}

#endif //ROBOTEAM_AI_ROLE_H
