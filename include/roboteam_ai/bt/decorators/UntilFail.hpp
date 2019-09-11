#pragma once

#include "../Decorator.hpp"

namespace bt {

class UntilFail : public Decorator {
public:
    Status update() override;
    std::string node_name() override { return "UntilFail"; };
};

} // bt
