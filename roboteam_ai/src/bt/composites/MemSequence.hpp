#pragma once

#include "../Composite.hpp"
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
};
} // bt
