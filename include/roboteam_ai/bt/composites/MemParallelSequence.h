#pragma once

#include "bt/Composite.hpp"

namespace bt {

    class MemParallelSequence : public Composite {
    public:
        explicit MemParallelSequence();
        virtual void initialize() override;
        Status update() override;
        std::string node_name() override { return "MemParallelSequence"; };
    private:
        std::map<Node::Ptr, Status> memory;
        int totalSuccess;
        int totalFailure;
    };

} // bt
