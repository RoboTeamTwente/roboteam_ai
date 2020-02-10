#pragma once

#include "bt/Decorator.h"

namespace bt {

class Succeeder : public Decorator {
   public:
    Status update() override;
    std::string node_name() override { return "Succeeder"; };
};

}  // namespace bt
