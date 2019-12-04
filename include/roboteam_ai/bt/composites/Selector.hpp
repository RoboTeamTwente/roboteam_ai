#pragma once

#include "bt/Composite.hpp"

namespace bt {

class Selector : public Composite {
public:
    Selector();

    Selector(nvector children);
    Status update() override;
    std::string node_name() override { return "Selector"; };
};
} // bt
