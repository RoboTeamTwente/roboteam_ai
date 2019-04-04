//
// Created by rolf on 3-4-19.
//

#ifndef ROBOTEAM_AI_DEFENDASSIGNCOACH_H
#define ROBOTEAM_AI_DEFENDASSIGNCOACH_H
#include "roboteam_utils/Vector2.h"
#include "roboteam_ai/src/utilities/World.h"
namespace rtt {
namespace ai {
namespace coach {
///This class keeps track of what all the defenders are doing and assigns them and communicates with them
class DefenceDealer {
    private:
        std::map<int, std::pair<Vector2, double>> defenderLocations;
        std::vector<int> defenders;
        bool doUpdate=true;
    public:

        void updateDefenderLocations();
        void addDefender(int id);
        void removeDefender(int id);
        std::shared_ptr<std::pair<Vector2, double>> getDefenderPosition(int id);
        void setDoUpdate();
        void visualizePoints();
};
}//coach
}//ai
}//rtt

#endif //ROBOTEAM_AI_DEFENDASSIGNCOACH_H
