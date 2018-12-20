#pragma once

#include "../Decorator.hpp"

namespace bt {

class UntilSuccess : public Decorator {
public:
    Status update() override;
    std::string node_name() override { return "UntilSuccess"; };
};

}
