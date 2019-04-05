//
// Created by baris on 2-4-19.
//

#include <roboteam_ai/src/world/World.h>
#include <roboteam_ai/src/world/Field.h>
#include "FormationCoach.h"

namespace rtt {
namespace ai {
namespace coach {
FormationCoach g_formation;

FormationCoach::FormationCoach() {
    done = false;
}

bool FormationCoach::isOffensiveStop(int ID) {
    if (! done) {
        makeActiveStopPositions();
        makeStopPositions();
        done = true;
    }

    for (auto robot : robotsStop) {
        if (robot.first == ID) {
            return robot.second;
        }
    }
    return false;
}

void FormationCoach::makeActiveStopPositions() {

    if (rtt::ai::world::world->getUs().size() < 3) {
        for (auto robot : rtt::ai::world::world->getUs()) {
            robotsStop[robot.id] = false;
        }
    }

    // get the TWO closest robots to the ball, so as much work as calling the world for it
    std::pair<int, double> closest = {- 1, 999};
    std::pair<int, double> closestest = {- 1, 999};
    Vector2 ballPos;
    if (rtt::ai::world::world->getBall())
        ballPos = rtt::ai::world::world->getBall()->pos;
    else
        ballPos = {0,0};
    for (auto robot : rtt::ai::world::world->getUs()) {
        double dist = (static_cast<Vector2>(robot.pos) - ballPos).length();
        if (dist < closest.second) {
            if (dist < closestest.second) {
                closest = closestest;
                closestest = {robot.id, dist};
            }
            else {
                closest = {robot.id, dist};
            }
        }
        robotsStop[robot.id] = false;
    }
    robotsStop[closest.first] = true;
    robotsStop[closestest.first] = true;

}
void FormationCoach::makeStopPositions() {
    int amount = 0;
    for (auto robot : robotsStop) {
        if (! robot.second)
            amount ++;
    }
    Vector2 startPoint = rtt::ai::world::field->get_field().left_penalty_line.begin;
    Vector2 endPoint = rtt::ai::world::field->get_field().left_penalty_line.begin;
    if (amount > 2) {
        auto size = ((startPoint - endPoint).length()/(amount - 2.0));
        Vector2 travel = (endPoint - startPoint).stretchToLength(size);
        for (int i = 0; i <= amount; i ++) {
            positionsStop.push_back((startPoint + (startPoint + travel*i)));
        }
    }
    else {
        // add them, if less is needed they will be looped anyways
        positionsStop.push_back(startPoint);
        positionsStop.push_back(endPoint);
    }

}
// TODO time
std::vector<Vector2> FormationCoach::getStopPositions() {
    makeStopPositions();
    return positionsStop;
}

}
}
}