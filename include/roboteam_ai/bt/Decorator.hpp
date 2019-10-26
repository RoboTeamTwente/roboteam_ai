#pragma once

#include "Node.hpp"

namespace bt {

class Decorator : public Node {
    public:
        void addChild(Node::Ptr child) override;

        std::vector<Node::Ptr> getChildren() override;

        bool HasNoChild() const;

        void terminate(Status s) override;

    protected:
        Node::Ptr child = nullptr;
};

} // bt
