#pragma once

#include "bt/Decorator.h"

namespace bt {

class UntilFail : public Decorator {
   public:
    Status update() override;
    std::string node_name() override { return "UntilFail"; };
};

}  // namespace bt
