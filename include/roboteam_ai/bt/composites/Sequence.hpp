#pragma once

#include "bt/Composite.hpp"

namespace bt {


class Sequence : public Composite {
public:
<<<<<<< HEAD
    Sequence() = default;
    Sequence(std::vector<std::shared_ptr<bt::Node>> children);
=======
    explicit Sequence() = default;
    explicit Sequence(const std::vector<std::shared_ptr<bt::Node>>& children);
>>>>>>> development

    Status update() override;

    std::string node_name() override { return "Sequence"; };
    };

} // bt
