//
// Created by baris on 2-4-19.
//

#ifndef ROBOTEAM_AI_FORMATIONCOACH_H
#define ROBOTEAM_AI_FORMATIONCOACH_H


#include <map>
namespace rtt {
namespace ai {
namespace coach {

class FormationCoach {
    public:
        explicit FormationCoach() = default;

        /// Stop ///
        bool isOffensiveStop(int ID);


    private:
        /// Stop ///
        std::map<int, bool> robots;
        bool formed = false;
        void formStop();



};

extern FormationCoach g_formation;

}
}
}

#endif //ROBOTEAM_AI_FORMATIONCOACH_H
