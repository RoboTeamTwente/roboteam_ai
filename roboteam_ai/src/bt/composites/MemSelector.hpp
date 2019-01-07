#pragma once

#include "../Composite.hpp"

namespace bt {

class MemSelector : public Composite {
private:
    size_t index = 0;
public:
    void initialize() override;
    Status update() override;
    std::string node_name() override { return "MemSelector"; };
};
} // bt
