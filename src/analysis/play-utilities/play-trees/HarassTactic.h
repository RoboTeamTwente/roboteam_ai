//
// Created by jessevw on 14.02.20.
//

#ifndef RTT_HARASSTACTIC_H
#define RTT_HARASSTACTIC_H

#include "bt/composites/Sequence.h"
#include "skills/gotopos/SkillGoToPos.h"
#include "skills/Halt.h"
namespace bt{
    class HarassTactic : public Sequence {
    public:
        HarassTactic();
        void build();
        Status update() override ;
    private:
        std::shared_ptr<rtt::ai::SkillGoToPos> gtp = nullptr;
        std::shared_ptr<rtt::ai::Halt> halt = nullptr;
    };
}



#endif //RTT_HARASSTACTIC_H
