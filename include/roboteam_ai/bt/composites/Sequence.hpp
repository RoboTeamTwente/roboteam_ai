#pragma once

#include "include/roboteam_ai/bt/Composite.hpp"

namespace bt {

class Sequence : public Composite {
public:
    Status update() override;
    std::string node_name() override { return "Sequence"; };
};

} // bt
