#pragma once

#include "../Decorator.hpp"

namespace bt {

class Repeater : public Decorator {
public:
    void initialize() override;
    Status update() override;
    std::string node_name() override { return "Repeater"; };

protected:
    int limit = 0;
    int counter = 0;
};

}
