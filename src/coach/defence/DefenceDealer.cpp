//
// Created by rolf on 3-4-19.
//

#include "coach/defence/DefenceDealer.h"
#include "interface/api/Input.h"

namespace rtt::ai::coach {
    DefenceDealer g_DefenceDealer;

    /// adds a defender to the available defendersList
    void DefenceDealer::addDefender(int id) {
        availableIDs.insert(id);
    }

    /// get the specific position of a defender with specified id
    std::optional<std::pair<Vector2, double>> DefenceDealer::getDefenderPosition(int id) {
        auto found = assignedDefenders.find(id);
        if (found == assignedDefenders.end()) {
            return std::nullopt;
        }

        return std::make_pair(found->second.targetPos, found->second.orientation);
    }

    void DefenceDealer::visualizePoints() {
        std::vector<Vector2> visualizationData;
        for (const auto &defender : assignedDefenders) {
            visualizationData.push_back(defender.second.targetPos);
        }
        ai::interface::Input::drawData(interface::Visual::DEFENSE, visualizationData, Qt::red, -1,
                                       ai::interface::Drawing::CIRCLES);
    }

    /// calculates the defender locations for all available defenders
    void DefenceDealer::updateDefenderLocations() {
        std::map<int, DefenderBot> lockedDefenders;
        std::set<int> freeDefenders;
        // LockedDefenders are defenders that are locked to a target for at least LOCKTIME ticks
        for (int id : availableIDs) {
            auto result = assignedDefenders.find(id);
            if (result == assignedDefenders.end()) {
                freeDefenders.emplace(id);
                continue;
            }

            if (result->second.coveredCount > LOCKTIME) {
                freeDefenders.emplace(result->first);
            } else {
                lockedDefenders.emplace(result->first, result->second);
            }
        }

        auto foundDefenders = g_defensivePositionCoach.decidePositions(lockedDefenders, freeDefenders);
        availableIDs.clear();
        if (foundDefenders.size() < freeDefenders.size() + lockedDefenders.size()) {
            std::cout << "[DefenceDealer] Warning: There is no opponent to defend against" << std::endl;
        }
        assignedDefenders = foundDefenders;
        visualizePoints();    //visualization
    }
} // rtt::ai::coach