#pragma once

#include "bt/Composite.hpp"

namespace bt {

class Selector : public Composite {
   public:
    explicit Selector() = default;
    explicit Selector(const std::vector<std::shared_ptr<bt::Node>>& children);
    Status update() override;
    std::string node_name() override { return "Selector"; };
};
}  // namespace bt
