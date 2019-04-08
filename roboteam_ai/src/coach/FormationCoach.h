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
        std::vector<Vector2> getStopPositions();
        void registerPassive(int ID);


    private:
        /// Stop ///
        std::vector<int> passiveRobots;
        std::vector<Vector2> passivePositions;
        std::vector<int> activeRobots;
        void makePassivePositions();
//        void makeActiveStopPositions();
        bool done = false;
        bool passiveDone = false;



};

extern FormationCoach g_formation;

}
}
}

#endif //ROBOTEAM_AI_FORMATIONCOACH_H
