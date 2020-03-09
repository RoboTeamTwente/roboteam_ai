//
// Created by jessevw on 06.12.19.
//

#ifndef RTT_PLAY_H
#define RTT_PLAY_H

#include <vector>
#include "bt/BehaviorTree.h"
#include "functional"
#include "world/World.h"

namespace rtt::ai::analysis {
using namespace rtt::ai::world;

    class Play {
    public:
        virtual bool isValidPlay(rtt::ai::world::World* world) const noexcept = 0;
        virtual uint8_t scorePlay(rtt::ai::world::World* world) const noexcept = 0;
        [[nodiscard]] virtual std::string_view getName() const noexcept = 0;
        [[nodiscard]] std::shared_ptr<bt::BehaviorTree> getTree() const noexcept;
    protected:
        std::shared_ptr<bt::BehaviorTree> tree;
        std::vector<std::unique_ptr<Role>> roles;
    };
}  // namespace rtt::ai::analysis

#endif  // RTT_PLAY_H
