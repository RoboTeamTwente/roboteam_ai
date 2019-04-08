//
// Created by baris on 2-4-19.
//

#include <roboteam_ai/src/world/World.h>
#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/utilities/RobotDealer.h>
#include "FormationCoach.h"

namespace rtt {
namespace ai {
namespace coach {
FormationCoach g_formation;

FormationCoach::FormationCoach() {
}

void FormationCoach::makePassivePositions() {

    if (passiveDone)
        return;

    int amount = passiveRobots.size();

    Vector2 startPoint = rtt::ai::world::field->get_field().left_penalty_line.begin;
    Vector2 endPoint = rtt::ai::world::field->get_field().left_penalty_line.end;
    if (amount > 2) {
        auto size = ((startPoint - endPoint).length()/(amount - 2.0));
        Vector2 travel = (endPoint - startPoint).stretchToLength(size);
        for (int i = 0; i <= amount; i ++) {
            passivePositions.push_back((startPoint + (startPoint + travel*i)));
        }
    }
    else {
        // add them, if less is needed they will be looped anyways
        passivePositions.push_back(startPoint);
        passivePositions.push_back(endPoint);
    }
    passiveDone = true;

}
std::vector<Vector2> FormationCoach::getStopPositions() {
    return passivePositions;
}
void FormationCoach::registerPassive(int ID) {
    passiveRobots.emplace_back(ID);

}
std::vector<int> FormationCoach::getPassiveRobots() {
    return passiveRobots;
}
void FormationCoach::terminate() {
    done = false;
    passiveDone = false;
    passiveRobots.clear();
    passivePositions.clear();
    activeRobots.clear();


}

}
}
}