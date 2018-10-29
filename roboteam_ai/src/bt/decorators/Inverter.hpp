#pragma once

#include "../Decorator.hpp"

namespace bt {

/*
    The Inverter decorator inverts the child node's status, i.e. failure becomes success and success becomes failure.
    If the child runs, the Inverter returns the status that it is running too.
*/
    class Inverter : public Decorator {
    public:
        Status Update() override;

        std::string node_name() override;
    };

}
