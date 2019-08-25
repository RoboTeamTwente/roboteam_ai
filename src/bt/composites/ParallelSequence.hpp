#pragma once

#include "../Composite.hpp"

namespace bt {

class ParallelSequence : public Composite {
public:
    explicit ParallelSequence(bool successOnAll = true, bool failOnAll = true);
    ParallelSequence(int minSuccess, int minFail);
    Status update() override;
    std::string node_name() override { return "ParallelSequence"; };
private:
    bool useSuccessFailPolicy = false;
    bool successOnAll = true;
    bool failOnAll = true;
    int minSuccess = 0;
    int minFail = 0;
};

} // bt
