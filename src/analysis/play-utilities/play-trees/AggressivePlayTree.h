//
// Created by jessevw on 13.02.20.
//

#ifndef RTT_AGGRESSIVEPLAYTREE_H
#define RTT_AGGRESSIVEPLAYTREE_H

#include "HarassTactic.h"
#include <bt/BehaviorTree.h>
namespace bt {
    class AggressivePlayTree : public BehaviorTree {
    public:
        AggressivePlayTree();
        Status update() override{};

    private:
        void build();
        void execute();
        void setParameters();
        HarassTactic harassTactic;
    };

}


#endif //RTT_AGGRESSIVEPLAYTREE_H
