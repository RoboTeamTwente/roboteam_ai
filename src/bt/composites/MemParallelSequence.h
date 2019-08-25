#pragma once

#include "../Composite.hpp"

namespace bt {

    class MemParallelSequence : public Composite {
    public:
        explicit MemParallelSequence();
        void initialize();
        Status update() override;
        std::string node_name() override { return "MemParallelSequence"; };
    private:
        std::map<Node::Ptr, Status> memory;
        int totalSuccess;
        int totalFailure;
    };

} // bt
