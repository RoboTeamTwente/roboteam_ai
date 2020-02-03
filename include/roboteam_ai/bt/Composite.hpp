#pragma once

#include "Node.hpp"

namespace bt {
/**
 * a type alias to create a vector of shared ptrs to nodes
 */
    using nvector = std::vector<std::shared_ptr<bt::Node>>;

    class Composite : public Node {
        public:
        /**
         * Constructor for composite where children vector is automatically added as children. The children are added from left to right,
         * with left getting ticked first
         * @param children
         */
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

}  // namespace bt
