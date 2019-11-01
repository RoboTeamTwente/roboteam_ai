//
// Created by jessevw on 01.11.19.
//

#ifndef RTT_KICKOFF_H
#define RTT_KICKOFF_H

#include "treeinterp/StrategyInterface.h"

namespace bt {
    class KickOff : public StrategyInterface {
    public:
        void createStrategy() override;

        void createTactics() override;

        KickOff();

    };
}

#endif //RTT_KICKOFF_H
