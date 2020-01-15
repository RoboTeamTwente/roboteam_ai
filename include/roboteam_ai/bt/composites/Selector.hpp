#pragma once

#include "bt/Composite.hpp"

namespace bt {

class Selector : public Composite {
public:
<<<<<<< HEAD
    Selector() = default;
    Selector(std::vector<std::shared_ptr<bt::Node>> children);
=======

    explicit Selector() = default;
    explicit Selector(const std::vector<std::shared_ptr<bt::Node>>& children);
>>>>>>> development
    Status update() override;
    std::string node_name() override { return "Selector"; };
};
} // bt
