#pragma once

#include "../Composite.hpp"
#include <string>

namespace bt {
/*
    The MemSequence composite ticks each child node in order, and remembers what child it prevously tried to tick.
    If a child fails or runs, the sequence returns the same status.
    In the next tick, it will try to run each child in order again.
    If all children succeeds, only then does the sequence succeed.
*/
class MemSequence : public Composite {
    public:
        size_t index;

        void initialize() override;

        Status update() override;

        std::string node_name() override;

        using Ptr = std::shared_ptr<MemSequence>;
};
} // bt
