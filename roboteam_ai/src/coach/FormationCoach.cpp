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


//void FormationCoach::makeActiveStopPositions() {
//
//    if (rtt::ai::world::world->getUs().size() < 3) {
//        return;
//    }
//
//    // get the TWO closest robots to the ball, so as much work as calling the world for it
//    std::pair<int, double> closest = {- 1, 999};
//    std::pair<int, double> closestest = {- 1, 999};
//    Vector2 ballPos;
//    if (rtt::ai::world::world->getBall())
//        ballPos = rtt::ai::world::world->getBall()->pos;
//    else
//        ballPos = {0,0};
//    for (const auto& robot : rtt::ai::world::world->getUs()) {
//
//        // Skip the keeper
//        if (robot.id == rtt::ai::robotDealer::RobotDealer::getKeeperID())
//            continue;
//
//        double dist = (static_cast<Vector2>(robot.pos) - ballPos).length();
//        if (dist < closest.second) {
//            if (dist < closestest.second) {
//                closest = closestest;
//                closestest = {robot.id, dist};
//            }
//            else {
//                closest = {robot.id, dist};
//            }
//        }
//    }
//    activeRobots.emplace_back(closest.first);
//    activeRobots.emplace_back(closestest.first);
//
//
//}
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
// TODO time
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