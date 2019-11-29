#pragma once

#include "bt/Composite.hpp"

namespace bt {

class Sequence : public Composite {
public:
    Sequence(std::vector<std::shared_ptr<bt::Node>> children);

    Status update() override;
    std::string node_name() override { return "Sequence"; };
};

} // bt
