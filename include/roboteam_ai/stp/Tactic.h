//
// Created by jessevw on 03.03.20.
//

#ifndef RTT_TACTIC_H
#define RTT_TACTIC_H

#include <vector>
#include "stp/StpInfo.h"
namespace rtt::ai::stp {
class Skill;

class Tactic {
   public:
    std::vector<Skill> skills;

    /**
     * Checks the skills vector to see which skill is currently running. Passes needed info to the skill
     * @param tacticInfo that contains the information needed to run the tactic
     */
    void updateActiveSkill(TacticInfo tacticInfo);

    /**
     * This function should calculate any extra information that the skills might need to be executed.
     * Things that should be calculated are for example how hard the kicker should shoot to get to the desired position
     * or how fast the dribbler should be spinning.
     *
     * Though this method is responsible for ensuring everything is calculated, it helps to use helpers so this
     * function doesn't become a massive hack
     */
    void calculateInfoForSkill();
};
}  // namespace rtt::ai::stp

#endif  // RTT_TACTIC_H
