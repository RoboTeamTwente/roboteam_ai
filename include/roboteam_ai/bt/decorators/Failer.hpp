#pragma once

#include "bt/Decorator.hpp"

namespace bt {

class Failer : public Decorator {
   public:
    Status update() override;
    std::string node_name() override { return "Failer"; };
};

}  // namespace bt
