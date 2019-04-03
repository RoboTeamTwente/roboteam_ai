//
// Created by baris on 2-4-19.
//

#ifndef ROBOTEAM_AI_FORMATIONCOACH_H
#define ROBOTEAM_AI_FORMATIONCOACH_H


#include <map>
#include <roboteam_utils/Vector2.h>
namespace rtt {
namespace ai {
namespace coach {

class FormationCoach {
    public:
        explicit FormationCoach();

        /// Stop ///
        bool isOffensiveStop(int ID);


    private:
        /// Stop ///
        // See if it is an active ball follower or just a passive defender
        std::map<int, bool> robotsStop;
        // Passive defender positions
        std::set<Vector2> positionsStop;
        void makeStopPositions();
        void makeActiveStopPositions();



};

extern FormationCoach g_formation;

}
}
}

#endif //ROBOTEAM_AI_FORMATIONCOACH_H
