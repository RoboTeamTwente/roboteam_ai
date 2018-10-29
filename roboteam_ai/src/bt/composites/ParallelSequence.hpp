#pragma once

#include "../Composite.hpp"

namespace bt {

    class ParallelSequence : public Composite {
    public:
        ParallelSequence(bool successOnAll = true, bool failOnAll = true);

        ParallelSequence(int minSuccess, int minFail);

        Status Update() override;

        std::string node_name() override;

        using Ptr = std::shared_ptr<ParallelSequence>;

    private:
        bool useSuccessFailPolicy = false;
        bool successOnAll = true;
        bool failOnAll = true;
        int minSuccess = 0;
        int minFail = 0;
    };

} // bt
