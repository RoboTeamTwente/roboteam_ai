//
// Created by jessevw on 06.12.19.
//

#ifndef RTT_PLAY_H
#define RTT_PLAY_H

#include <vector>
#include "bt/BehaviorTree.hpp"
#include "functional"

namespace rtt::ai::analysis {
/**
 * The play has a vector of invariants, when the invariants are false the play is abandoned.
 *
 * TODO: Should this object have a behaviourtree associated to it? I think yes
 */
class Play {
   public:
    Play();

    Play(std::string name, std::vector<std::function<bool(world::World*, world::Field*)>> invariants);
    void setInvariants(const std::vector<std::function<bool(world::World*, world::Field*)>>& invariants);
    const std::vector<std::function<bool(world::World*, world::Field*)>>& getInvariants() const;

    /**
     *
     * @return true if all the invariants of this strategy are true
     */
    bool isValidPlay(rtt::ai::world::World* world, rtt::ai::world::Field* field);
    // TODO: Move this to the derived class
    int scorePlay(world::World* world, world::Field* field) {return 1;};
    std::string getName();

   protected:
    std::vector<std::function<bool(world::World*, world::Field*)>> invariants;
    std::shared_ptr<bt::BehaviorTree> tree;
public:
    const std::shared_ptr<bt::BehaviorTree> &getTree() const;

protected:
    std::string name;
};
}  // namespace rtt::ai::analysis

#endif  // RTT_PLAY_H
