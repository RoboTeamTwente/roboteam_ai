//
// Created by rolf on 3-4-19.
//

#ifndef ROBOTEAM_AI_DEFENDASSIGNCOACH_H
#define ROBOTEAM_AI_DEFENDASSIGNCOACH_H

#include "world/World.h"
#include "DefencePositionCoach.h"
#include "roboteam_utils/Vector2.h"

#include <set>
#include <map>

namespace rtt::ai::coach {

///This class keeps track of what all the defenders are doing and assigns them and communicates with them
    class DefenceDealer {
    private:
        constexpr static int LOCKTIME = 18;
        std::map<int, DefenderBot> assignedDefenders;
        std::set<int> availableIDs;
    public:
        void updateDefenderLocations();

        void addDefender(int id);

        [[nodiscard]] std::optional<std::pair<rtt::Vector2, double>> getDefenderPosition(int id);

        void visualizePoints();
    };

    extern DefenceDealer g_DefenceDealer;


} // rtt::ai::coach

#endif //ROBOTEAM_AI_DEFENDASSIGNCOACH_H
