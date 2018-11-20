#pragma once

#include "Node.hpp"

namespace bt {

class Composite : public Node {
    public:
        virtual ~Composite();

        void AddChild(Node::Ptr child) override;

        bool HasNoChildren() const;

        void Terminate(Status s) override;

    protected:
        std::vector<Node::Ptr> children;
        size_t index = 0;
};

}
