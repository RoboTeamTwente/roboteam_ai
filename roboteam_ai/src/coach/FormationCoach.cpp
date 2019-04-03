//
// Created by baris on 2-4-19.
//

#include "FormationCoach.h"

namespace rtt {
namespace ai {
namespace coach {
FormationCoach g_formation;
bool FormationCoach::isOffensiveStop(int ID) {

    if (!formed) {
        formStop();
    }
    for (auto robot : robots) {
        if (robot.first == ID) {
            return robot.second;
        }
    }
    return false;
}

void FormationCoach::formStop() {

}
}
}
}