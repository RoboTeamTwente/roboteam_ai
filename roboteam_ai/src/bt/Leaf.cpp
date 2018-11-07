#include <memory>

#include "Node.hpp"
#include "Leaf.hpp"

namespace bt {


Leaf::Leaf(std::string name, Blackboard::Ptr blackboard) {
    setProperties(blackboard);
    setName(name);

}
void Leaf::setName(std::string newName) {
    name = std::move(newName);

}

}
