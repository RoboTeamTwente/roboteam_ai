#pragma once

#include "bt/Composite.hpp"
namespace bt {

class MemSelector : public Composite {
    FRIEND_TEST(BehaviorTreeTest, selectorComposites);
    /**
     * Constructor with parameter of the children for the mem selector
     * @param children nodes that must be given in order from left to right,
     * left getting ticked first
     */
    MemSelector(nvector children);

    MemSelector();
private:

    size_t index = 0;
public:
    void initialize() override;
    Status update() override;
    std::string node_name() override { return "MemSelector"; };
};
} // bt
