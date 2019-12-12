#pragma once

#include "bt/Composite.hpp"

namespace bt {

class Selector : public Composite {
public:
    Selector() = default;

    Selector(std::vector<std::shared_ptr<bt::Node>> children);
    Status update() override;
    std::string node_name() override { return "Selector"; };
};
} // bt
