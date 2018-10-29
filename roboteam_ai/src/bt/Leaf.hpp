#pragma once

#include "Node.hpp"
#include "Blackboard.hpp"
#include <memory>

namespace bt {
//TODO: implement node_name() functionality? Can perhaps also be done elsewhere
    class Leaf : public Node {
    public:
        Leaf();

        virtual ~Leaf() = default;

        Leaf(Blackboard::Ptr blackboard);

        Leaf(std::string name, Blackboard::Ptr blackboard);

        void SetBlackboard(Blackboard::Ptr blackboard);

        virtual Status Update() = 0;

        const std::string name;

    protected:
        Blackboard::Ptr blackboard;
    };

}
