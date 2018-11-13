#pragma once

#include "../Decorator.hpp"

namespace bt {

/*
    The Repeater decorator repeats infinitely or to a limit until the child returns success.
*/
class Repeater : public Decorator {
    public:

        void Initialize() override;

        Status Update() override;

        std::string node_name() override;

    protected:
        int limit;
        int counter = 0;
};

}
