#pragma once

#include "Composite.h"

namespace bt {

class Sequence : public Composite {
   public:
    explicit Sequence() = default;
    explicit Sequence(const std::vector<std::shared_ptr<bt::Node>> &children);
    Status update() override;

    std::string node_name() override { return "Sequence"; };
};

}  // namespace bt
