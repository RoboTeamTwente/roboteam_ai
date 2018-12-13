#pragma once

#include "../Composite.hpp"

namespace bt {

/*
    The Selector composite ticks each child node in order.
    If a child succeeds or runs, the sequence returns the same status.
    In the next tick, it will try to run each child in order again.
    If all children fails, only then does the selector fail.
*/
class Selector : public Composite {
    public:
        Status update() override;

        std::string node_name() override;

        using Ptr = std::shared_ptr<Selector>;
};
} // bt
