//
// Created by rolf on 3-4-19.
//

#include "DefenceDealer.h"
#include "roboteam_ai/src/interface/api/Input.h"

namespace rtt {
namespace ai {
namespace coach {

DefenceDealer g_DefenceDealer;

/// adds a defender to the available defendersList
void DefenceDealer::addDefender(int id) {
    bool robotIsRegistered = std::find(availableIDs.begin(), availableIDs.end(), id) != availableIDs.end();
    if (! robotIsRegistered) {
        availableIDs.push_back(id);
    }
}

/// get the specific position of a defender with specified id
std::shared_ptr<std::pair<Vector2, double>> DefenceDealer::getDefenderPosition(int id) {
    for (const auto &defender: assignedDefenders) {
        if (defender.id == id) {
            std::pair<Vector2, double> pos = std::make_pair(defender.targetPos, defender.orientation);
            return std::make_shared<std::pair<Vector2, double>>(pos);
        }
    }
    return nullptr;
}
void DefenceDealer::visualizePoints() {
    std::vector<Vector2> visualizationData;
    for (const auto &defender : assignedDefenders) {
        visualizationData.emplace_back(defender.targetPos);
    }
    ai::interface::Input::drawData(interface::Visual::DEFENSE, visualizationData, Qt::red, - 1,
            ai::interface::Drawing::CIRCLES);
}
/// calculates the defender locations for all available defenders
void DefenceDealer::updateDefenderLocations() {
    std::vector<DefenderBot> lockedDefenders;
    std::vector<int> freeDefenders;
    // LockedDefenders are defenders that are locked to a target for atleast LOCKTIME ticks
    for (int id :availableIDs) {
        bool idFound = false;
        for (const auto &defender: assignedDefenders) {
            if (id == defender.id) {
                defender.coveredCount>LOCKTIME ? lockedDefenders.push_back(defender) : freeDefenders.push_back(defender.id);
                idFound = true;
                break;
            }
        }
        if (! idFound) {
            freeDefenders.push_back(id);
        }
    }
    availableIDs.clear();
    assignedDefenders = g_defensivePositionCoach.decidePositions(lockedDefenders, freeDefenders);
    visualizePoints();    //visualization
}
}//coach
}//ai
}//rtt
