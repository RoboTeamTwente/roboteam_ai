//
// Created by rolf on 3-4-19.
//

#ifndef ROBOTEAM_AI_DEFENDASSIGNCOACH_H
#define ROBOTEAM_AI_DEFENDASSIGNCOACH_H

#include "include/roboteam_ai/world/World.h"
#include "DefencePositionCoach.h"
#include "Vector2.h"

namespace rtt {
namespace ai {
namespace coach {
///This class keeps track of what all the defenders are doing and assigns them and communicates with them
class DefenceDealer {
    private:
        const int LOCKTIME=18;
        std::vector<DefenderBot> assignedDefenders;
        std::vector<int> availableIDs;
    public:
        void updateDefenderLocations();
        void addDefender(int id);
        std::shared_ptr<std::pair<rtt::Vector2, double>> getDefenderPosition(int id);
        void visualizePoints();
};
extern DefenceDealer g_DefenceDealer;

}//coach
}//ai
}//rtt

#endif //ROBOTEAM_AI_DEFENDASSIGNCOACH_H
