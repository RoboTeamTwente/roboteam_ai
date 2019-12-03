#pragma once

#include "Node.hpp"

namespace bt {
    using nvector = std::vector<std::shared_ptr<bt::Node>>;


    class Composite : public Node {
    public:
        Composite(std::vector<std::shared_ptr<bt::Node>> children);

        Composite();

        void addChild(Node::Ptr child) override;

        std::vector<Node::Ptr> getChildren() override;

        bool HasNoChildren() const;

        void terminate(Status s) override;

    protected:
        std::vector<Node::Ptr> children;
        size_t index = 0;
    };

}
