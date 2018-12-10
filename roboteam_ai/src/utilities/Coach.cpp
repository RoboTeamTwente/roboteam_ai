//
// Created by baris on 6-12-18.
//
#include "Coach.h"

namespace rtt {
namespace ai {
namespace coach {

    std::map<int, int> Coach::defencePairs;

int Coach::pickOffensivePassTarget(int selfID, std::string roleName) {

    // Get the other robots in that tactic
    std::string tacticName = dealer::getTacticNameForRole(std::move(roleName));
    auto tacticMates = dealer::findRobotsForTactic(tacticName);

    // Pick a free one TODO make better
    for (auto r : tacticMates) {
        if (r != selfID) {
            if (control::ControlUtils::hasClearVision(selfID, r, World::get_world(), 2)) {
                return r;
            }
        }
    }
    return - 1;
}
int Coach::pickDefensivePassTarget(int selfID) {

    auto world = World::get_world();
    auto us = world.us;
    int safelyness = 3;
    while (safelyness >= 0) {
        for (auto friendly : us) {
            if (control::ControlUtils::hasClearVision(selfID, friendly.id, world, safelyness)) {
                return friendly.id;
            }
        }
        safelyness --;
    }
    return - 1;
}

int Coach::pickOpponentToCover(int selfID) {

    dangerfinder::DangerData DangerData = dangerfinder::DangerFinder::instance().getMostRecentData();
    std::vector<int> dangerList = DangerData.dangerList;
    for(int& opponentID : dangerList) {
        if(defencePairs.find(opponentID) == defencePairs.end()) {
            defencePairs.insert({opponentID, selfID});
            return opponentID;
        }
    }

    return -1;
}

}
}
}
