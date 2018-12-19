#pragma once

#include "../Decorator.hpp"

namespace bt {

/*
    The UntilSuccess decorator repeats until the child returns success and then returns success.
*/
class UntilSuccess : public Decorator {
    public:
        Status update() override;

        std::string node_name() override;
};

}
