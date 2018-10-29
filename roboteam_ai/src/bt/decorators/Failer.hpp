#pragma once

#include "../Decorator.hpp"

namespace bt {

/*
    The Failer decorator returns failure, regardless of what happens to the child.
*/
    class Failer : public Decorator {
    public:
        Status Update() override;

        std::string node_name() override;
    };

}
