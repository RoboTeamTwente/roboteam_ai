#pragma once

#include "bt/Composite.hpp"
#include <string>
#include <iostream>

namespace bt {

class MemSequence : public Composite {
private:

    size_t index = 0;

public:
    void initialize() override;
    Status update() override;
    void terminate(Status s) override;
    std::string node_name() override { return "MemSequence"; };
    MemSequence();
    /**
     * constructor to initialize the children of the mem sequence
     * @param children of the memsequence, must be given in order from left to right,
     * with left getting ticked first.
     */
    MemSequence(nvector children);


};
} // bt
