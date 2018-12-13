#pragma once

#include "Node.hpp"

namespace bt {

class Composite : public Node {
    public:
        void addChild(Node::Ptr child) override;

        std::vector<Node::Ptr> getChildren() override;

        bool HasNoChildren() const;

        void terminate(Status s) override;

    protected:
        std::vector<Node::Ptr> children;
        size_t index = 0;
};

}
