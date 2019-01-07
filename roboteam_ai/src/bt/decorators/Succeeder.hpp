#pragma once

#include "../Decorator.hpp"

namespace bt {

class Succeeder : public Decorator {
public:
    Status update() override;
    std::string node_name() override { return "Succeeder"; };
};

}
