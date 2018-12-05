#pragma once

#include "Node.hpp"

namespace bt {
//TODO: implement node_name() functionality? Can perhaps also be done elsewhere
class Leaf : public Node {
    public:
        Leaf() = default;
        virtual ~Leaf() = default;
        Leaf(std::string name, Blackboard::Ptr blackboard);
        virtual Status update() = 0;
        void setName(std::string);
        std::string name;
};

}
