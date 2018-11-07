#pragma once

#include "Node.hpp"
#include "Blackboard.hpp"
#include <memory>

namespace bt {
//TODO: implement node_name() functionality? Can perhaps also be done elsewhere
class Leaf : public Node {
    public:

        virtual ~Leaf() = default;

        Leaf(std::string name, Blackboard::Ptr blackboard);

        virtual Status Update() = 0;

        void setName(std::string);

        std::string name;

};

}
