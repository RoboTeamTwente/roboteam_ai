#pragma once

#include "bt/Composite.hpp"

namespace bt {

class Selector : public Composite {
public:
    Status update() override;
    std::string node_name() override { return "Selector"; };
};
} // bt
