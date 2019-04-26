//
// Created by thijs on 12-4-19.
//

#include "HarassRobotCoach.h"
#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/world/World.h>
#include "../../world/WorldData.h"
#include <algorithm> //sort
#include <roboteam_ai/src/analysis/AnalysisReport.h>
#include <roboteam_ai/src/analysis/GameAnalyzer.h>

namespace rtt {
namespace ai {
namespace coach {

HarassRobotCoach g_harassRobotCoach;

// update harass-targetPosition based on current position and the position of other robots in the same tactic
// myIndex is used to keep track of the position of this robot in the static array
Vector2 HarassRobotCoach::getHarassPosition(const RobotPtr &thisRobot, int &myIndex) {
    ball = world::world->getBall();
    auto field = world::field->get_field();
    Vector2 currentLocation = thisRobot->pos;
    auto report = analysis::GameAnalyzer::getInstance().getMostRecentReport();
    auto ballPossession = report->ballPossession;
    if (ballPossession == analysis::WE_HAVE_BALL) {
        bestXPos = 1.2;
    } else if (ballPossession == analysis::THEY_HAVE_BALL) {
        bestXPos = std::min(0.67 * field.field_length, 0.67 * (ball->pos.x + field.field_width));
    } else {
        bestXPos = 0.0;
    }

    // initialize
    if (myIndex == - 1) {
        return initialize(currentLocation, myIndex);
    }

    // update location
    currentRobotPositions[myIndex] = currentLocation;

    // find something to do (harass robot, block ball line, etc..)
    // if ball is on our side, annoy opponents that want to get to our side
    // if we have the ball, stand free to receive it

    // if ball is on their side, annoy robots that want to go to their side

    // if ball on their side
    if (ball->pos.x > 0) {
        return getHarassPositionWhereTheyHaveBall(thisRobot, myIndex);

    // else, ball is on our side
    } else {
        return getHarassPositionWhereTheyHaveBall(thisRobot, myIndex);
    }

    return standInMidField(thisRobot, myIndex);
}

int HarassRobotCoach::getRobotIndexCloseToEnemyRobot(const world::World::RobotPtr &enemyRobot) const {
    double closestDistanceSquared = 9e9;
    int bestIndex = - 1;
    for (unsigned int i = 0; i < currentRobotPositions.size(); i ++) {
        auto &robotPos = currentRobotPositions[i];
        double lengthSquared = (robotPos - enemyRobot->pos).length2();
        if (lengthSquared < closestDistanceSquared) {
            closestDistanceSquared = lengthSquared;
            bestIndex = i;
        }
    }
    return bestIndex;
}

Vector2 HarassRobotCoach::harassRobot(int myIndex, int id, bool stayInMidField) {
    // (re)set the robot harassed to the robot with that id
//    targetRobotsToHarass[myIndex] = RobotPtr(nullptr);
//    RobotPtr robotToHarass = world::world->getRobotForId(id, false);
//    if (! robotToHarass) return {currentRobotPositions[myIndex]};
//    if (! ball) return {robotToHarass->pos.x - 0.3, robotToHarass->pos.y};
//
//    // set target
//    Vector2 target;
//    if (robotToHarass->getDistanceToBall() >= 0.0) {
//        target = ball->pos + (ball->pos - robotToHarass->pos).stretchToLength(0.2);
//    }
//    else {
//        double a = 0.74;
//        target = robotToHarass->pos*a + ball->pos*(1 - a);
//        target.x = target.x > bestXPos + 1.0 ? target.x*a + (bestXPos + 1.0)*(1 - a) : target.x;
//        target.x = target.x < bestXPos - 1.0 ? target.x*a + (bestXPos - 1.0)*(1 - a) : target.x;
//    }
//
//    targetRobotPositions[myIndex] = target;
//    targetRobotsToHarass[myIndex] = robotToHarass;
//    for (int i = 0; i < static_cast<int>(targetRobotsToHarass.size()); i ++) {
//        if (i == myIndex) continue;
//        auto &targetRobot = targetRobotsToHarass[i];
//        if (! targetRobot) continue;
//
//        if (targetRobot->id == robotToHarass->id) {
//            targetRobot = RobotPtr(nullptr);
//        }
//    }

    Vector2 target;
    RobotPtr robotToHarass = world::world->getRobotForId(id, false);

    if (robotToHarass->vel.length() > MINIMUM_HARASS_VELOCITY) {
        Vector2 harassingDistance = {robotToHarass->vel.length() * HARASSER_SECONDS_AHEAD, 0};
        target = robotToHarass->pos + harassingDistance.rotate(robotToHarass->vel.toAngle());
    } else {
        target = robotToHarass->pos;
        target.x -= DEFAULT_HARASSING_DISTANCE;
    }

    Vector2 projection = currentRobotPositions[myIndex].project(robotToHarass->pos, target);
    if ((projection - currentRobotPositions[myIndex]).length() < 0.05) {
        target = currentRobotPositions[myIndex];
    }

    return target;
}

Vector2 HarassRobotCoach::standFree(const RobotPtr &ourRobotWithBall, const RobotPtr &thisRobot, int myIndex) {
    Vector2 currentLocation = thisRobot->pos;
    Vector2 passLine = ourRobotWithBall->pos - currentLocation;
    Angle passAngle = passLine.toAngle();
    Angle smallestAngle = Angle(M_PI);
    auto enemyRobots = world::world->getThem();
    for (auto &robot : enemyRobots) {
        Vector2 blockLine = ourRobotWithBall->pos - robot.pos;
        Angle blockAngle = blockLine.toAngle();
        Angle deltaAngle = blockAngle - passAngle;
        if (abs(deltaAngle.getAngle()) < abs(smallestAngle.getAngle())) {
            auto distToLine = control::ControlUtils::distanceToLine(robot.pos, ourRobotWithBall->pos, currentLocation);
            auto distToLineWithEnds = control::ControlUtils::distanceToLineWithEnds(robot.pos, ourRobotWithBall->pos,
                    currentLocation);
            if (distToLine == distToLineWithEnds) {
                smallestAngle = deltaAngle;
            }
        }
    }
    Vector2 target;
    Vector2 closeRobotPos = Vector2(42.0, 42.0);
    for (auto &otherRobot : world::world->getAllRobots()) {
        if (otherRobot.id == thisRobot->id && otherRobot.team == thisRobot->team) continue;

        if ((otherRobot.pos - currentLocation).length2() < 0.5) {
            closeRobotPos = otherRobot.pos;
        }
    }
    if (closeRobotPos != Vector2(42.0, 42.0)) {
        target = currentLocation + (currentLocation - closeRobotPos).normalize();
    }
    else if (abs(smallestAngle) > 0.25) {
        target = currentLocation;
    }
    else if (smallestAngle.getAngle() > 0.0) {
        target = currentLocation + passLine.rotate(M_PI_2).normalize();
    }
    else {
        target = currentLocation - passLine.rotate(M_PI_2).normalize();
    }

    target.x = target.x < bestXPos ? bestXPos : target.x;
    targetRobotPositions[myIndex] = target;
    targetRobotsToHarass[myIndex] = RobotPtr(nullptr);
    return target;
}

Vector2 HarassRobotCoach::initialize(const Vector2 &currentLocation, int &myIndex) {
    Vector2 target = {bestXPos, currentLocation.y};
    currentRobotPositions.push_back(currentLocation);
    targetRobotPositions.push_back(target);
    targetRobotsToHarass.push_back(RobotPtr(nullptr));
    myIndex = currentRobotPositions.size() - 1; // set my index (side-effect, but works well tm)
    return target;
}

Angle HarassRobotCoach::getHarassAngle(const HarassRobotCoach::RobotPtr &thisRobot, int &myIndex) {
    ball = world::world->getBall();

    // if there is a robot to harass
    if (targetRobotsToHarass[myIndex]) {
        auto robotToHarass = targetRobotsToHarass[myIndex];
        if (robotToHarass->hasBall()) {
            return (ball->pos - thisRobot->pos).toAngle();
        }
        else {
            return (robotToHarass->pos - thisRobot->pos).toAngle();
        }
    }
    // else, aim towards the ball
    return (ball->pos - thisRobot->pos).toAngle();
}

Vector2 HarassRobotCoach::getHarassPositionWhereTheyHaveBall(const RobotPtr &thisRobot, int &myIndex) {
    // check opponent closest to ball and closest to harasser
    auto closestRobotToBallDistance = DBL_MAX;
    RobotPtr closestRobotToBall;

    auto closestRobotToHarasserDistance = DBL_MAX;
    RobotPtr closestRobotToHarasser;

    for (auto robot : world::world->getThem()) {
        // check if robot is closest to ball
        double distanceToBall = (ball->pos - robot.pos).length();
        if (distanceToBall < closestRobotToBallDistance) {
            closestRobotToBallDistance = distanceToBall;
            closestRobotToBall = std::make_shared<Robot>(robot);
        }

        // check if robot is closest to the harasser
        double distanceToHarasser = (thisRobot->pos - robot.pos).length();
        if (distanceToHarasser < closestRobotToHarasserDistance) {
            closestRobotToHarasserDistance = distanceToHarasser;
            closestRobotToHarasser = std::make_shared<Robot>(robot);
        }
    }

    // see if opponent is too close to the ball
    if (closestRobotToBallDistance <= TOO_CLOSE_TO_BALL_DISTANCE) {
        // if this robot is closest to the robot with ball, harass that robot
        int bestIndex = getRobotIndexCloseToEnemyRobot(closestRobotToBall);
        if (bestIndex == myIndex) {
            return harassRobot(myIndex, closestRobotToBall->id, false);
        }
    }

    // else, harass the opponent closest to the harasser if it is close enough to the middle

    if (abs(closestRobotToHarasser->pos.x) <= HARASS_THRESHOLD) {
        return harassRobot(myIndex, closestRobotToHarasser->id);
    }

    // else, get a new robot to harass



//    auto lambdaBall = ball;
//    std::vector<Robot> theirRobots = world::world->getThem();
//    std::sort(theirRobots.begin(), theirRobots.end(),
//            [lambdaBall](const Robot &a, const Robot &b) -> bool {
//              // sorts by distance of enemy robot to this robot
//              return (a.pos - lambdaBall->pos).length2() < (b.pos - lambdaBall->pos).length2();
//            });
//
//    for (auto &robot : theirRobots) {
//        bool robotIsHarassed = false;
//        for (int i = 0; i < static_cast<int>(targetRobotsToHarass.size()); i ++) {
//            if (i == myIndex) continue;
//
//            auto &harassTarget = targetRobotsToHarass[i];
//            if (harassTarget) {
//                if (robot.id == harassTarget->id) {
//                    robotIsHarassed = true;
//                    break;
//                }
//            }
//        }
//        if (! robotIsHarassed) return harassRobot(myIndex, robot.id);
//    }

    // can't get a robot to harass, remove my targetRobot if it was there
    targetRobotsToHarass[myIndex] = RobotPtr(nullptr);

    return standInMidField(thisRobot, myIndex);
}
Vector2 HarassRobotCoach::getHarassPositionWhereWeHaveBall(const HarassRobotCoach::RobotPtr &robotWithBall,
        const HarassRobotCoach::RobotPtr &thisRobot, int &myIndex) {

    return standFree(robotWithBall, thisRobot, myIndex);
}

Vector2 HarassRobotCoach::standInMidField(const HarassRobotCoach::RobotPtr &thisRobot, int &myIndex) {
    Vector2 currentLocation = thisRobot->pos;

    // if not in the right x-location yet, go there first
    if (abs(currentLocation.x - bestXPos) > 0.5) {
        Vector2 target = {bestXPos, currentLocation.y};
        targetRobotPositions[myIndex] = target;
        return target;
    }

    // if you have nothing else to do, just don't get too close to other harass-robots
    for (int i = 0; i < static_cast<int>(currentRobotPositions.size()); i ++) {
        if (i == myIndex) continue;

        auto &robotPos = currentRobotPositions[i];
        if ((robotPos - currentLocation).length() < MIN_DISTANCE_BETWEEN_MIDFIELDERS) {
            Vector2 target = {currentLocation.x, currentLocation.y +
                    (double) (robotPos.y > currentLocation.y ? MIN_DISTANCE_BETWEEN_MIDFIELDERS*- 1.2 : MIN_DISTANCE_BETWEEN_MIDFIELDERS*1.2)};
            targetRobotPositions[myIndex] = target;

            if (!world::field->pointIsInField(target, DISTANCE_FROM_SIDES)) {
                auto field = world::field->get_field();
                if (target.y > 0) {
                    target.y = field.field_width / 2 - DISTANCE_FROM_SIDES;
                } else {
                    target.y = -field.field_width / 2 + DISTANCE_FROM_SIDES;
                }
            }

            return target;
        }
    }

    return currentLocation;
}

} //coach
} //ai
} //rtt
