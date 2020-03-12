//
// Created by robzelluf on 5/27/19.
//

#include <include/roboteam_ai/world_new/World.hpp>
#include <roboteam_utils/Print.h>
#include <include/roboteam_ai/coach/midfield/MidFieldCoach.h>

namespace rtt::ai::coach {

MidFieldCoach g_midFieldCoach;

void MidFieldCoach::addMidFielder(world_new::view::RobotView thisRobot) {
        Vector2 target = thisRobot->getPos();
        currentMidfielders_new.push_back(thisRobot);
        targetPositions[thisRobot->getId()] = target;
}

MidFieldCoach::Target MidFieldCoach::getTargetPosition(const Field &field, world_new::view::RobotView thisRobot) {
    // Check if already having an opponent as target
    if (targetRobotsToHarass_new.find(thisRobot->getId()) != targetRobotsToHarass_new.end()) {
        world_new::view::RobotView opponent = world_new::World::instance()->getWorld()->getRobotForId(thisRobot->getId()).value();
        if (validOpponent(field, opponent)) {
            return harassRobot(thisRobot, opponent, HARASS_DEFENSIVE);
        } else {
            targetRobotsToHarass.erase(thisRobot->getId());
        }
    }

    // If no opponent, or opponent not valid anymore, pick new target
    world_new::view::RobotView opponent = findRobotToHarass(field, thisRobot);

    // If there is a opponent to harass
    if (opponent) {
        HarassType harassType = getHarassType(thisRobot, opponent);

        switch (harassType) {
            case HARASS_OFFENSIVE: {
                return harassRobot(thisRobot, opponent, HARASS_OFFENSIVE);
            }
            case HARASS_DEFENSIVE: {
                return harassRobot(thisRobot, opponent, HARASS_DEFENSIVE);
            }
            case BLOCK_PASS: {
                return blockPass(thisRobot, opponent, world_new::World::instance()->getWorld()->getBall().value());
            }
            case BALL: {
                return getBall(thisRobot, opponent);
            }
            case STAND_FREE: {
                return standFree(field, thisRobot);
            }
        }
    } else {
        return standFree(field, thisRobot);
    }
}

bool MidFieldCoach::validOpponent(const Field &field, const world_new::view::RobotView opponent) {
    if (abs(opponent->getPos().x) > DISTANCE_FROM_MIDDLE_LINE) {
        return false;
    }
    return FieldComputations::pointIsInField(field, opponent->getPos());
}

void MidFieldCoach::removeMidFielder(world_new::view::RobotView thisRobot) {
//TODO fix this
//    currentMidfielders_new.erase(std::remove(currentMidfielders_new.begin(), currentMidfielders_new.end(), thisRobot), currentMidfielders_new.end());
RTT_ERROR("removing midfielders does not work yet!")

    if (targetPositions.find(thisRobot->getId()) != targetPositions.end()) {
        targetPositions.erase(thisRobot->getId());
    }

    if (targetRobotsToHarass.find(thisRobot->getId()) != targetRobotsToHarass.end()) {
        targetRobotsToHarass.erase(thisRobot->getId());
    }
}

MidFieldCoach::Target MidFieldCoach::blockPass(const world_new::view::RobotView thisRobot, const world_new::view::RobotView opponent, const world_new::view::BallView ball) const {
    Target target;
    Vector2 projectionPoint = thisRobot->getPos().project(ball->getPos(), opponent->getPos());
    if (control::ControlUtils::isPointProjectedOnLineSegment(projectionPoint, opponent->getPos(), ball->getPos())) {
        target.targetPosition = projectionPoint;
    } else {
        target.targetPosition = ball->getPos();
    }
    return target;
}
    MidFieldCoach::Target MidFieldCoach::harassRobot(const world_new::view::RobotView thisRobot, const world_new::view::RobotView opponent, HarassType harassType) const {
        Target target;
        target.targetRobot = opponent->getId();
        if (opponent->getVel().length() < MIN_OPPONENT_VELOCITY) {
            target = harassSlowRobot(opponent, harassType, target);
        } else {
            target = harassFastRobot(thisRobot, opponent, target);
        }
        return target;
    }

    MidFieldCoach::Target &MidFieldCoach::harassFastRobot(const world_new::view::RobotView thisRobot, const world_new::view::RobotView opponent, MidFieldCoach::Target &target) const {
        Vector2 futureOpponentPos = opponent->getPos() + opponent->getVel() * HARASSER_SECONDS_AHEAD;
        Vector2 projectionPoint = thisRobot->getPos().project(opponent->getPos(), futureOpponentPos);

        // Check if the projection is actually between the opponent and it's future position
        // If not, move to the future position
        if (control::ControlUtils::isPointProjectedOnLineSegment(projectionPoint, opponent->getPos(), futureOpponentPos)) {
            if ((projectionPoint - thisRobot->getPos()).length() < STAND_STILL_DISTANCE) {
                target.targetPosition = thisRobot->getPos();
            } else {
                target.targetPosition = projectionPoint;
            }
        } else {
            target.targetPosition = futureOpponentPos;
        }
        return target;
    }

MidFieldCoach::Target &MidFieldCoach::harassSlowRobot(const world_new::view::RobotView opponent, const MidFieldCoach::HarassType &harassType, MidFieldCoach::Target &target) const {
    if (harassType == HARASS_DEFENSIVE) {
        target.targetPosition = opponent->getPos() - Vector2{DEFAULT_HARASS_DISTANCE, 0};
    } else {
        target.targetPosition = opponent->getPos() + Vector2{DEFAULT_HARASS_DISTANCE, 0};
    }
    return target;
}

world_new::view::RobotView MidFieldCoach::findRobotToHarass(const Field &field, const world_new::view::RobotView thisRobot) {
    world_new::view::RobotView closestRobot{nullptr};
    auto shortestDistance = 9e9;

    // Loop over all opponents to find a opponent to harass
    for (const auto &opponent : world_new::World::instance()->getWorld()->getThem()) {
        // If opponent is not valid, ignore it
        if (!validOpponent(field, opponent)) continue;

        bool alreadyBeingHarassed = isRobotAlreadyBeingHarassed(opponent);

        if (!alreadyBeingHarassed) {
            double distance = (opponent->getPos() - thisRobot->getPos()).length();
            if (distance < shortestDistance) {
                closestRobot = opponent;
                shortestDistance = distance;
            }
        }
    }

    return closestRobot;
}

bool MidFieldCoach::isRobotAlreadyBeingHarassed(const world_new::view::RobotView opponent) const {
    bool alreadyBeingHarassed = false;
    for (const auto &currentOpponent : targetRobotsToHarass) {
        if (opponent->getId() == currentOpponent.second->getId()) {
            alreadyBeingHarassed = true;
            break;
        }
    }
    return alreadyBeingHarassed;
}

MidFieldCoach::Target MidFieldCoach::standFree(const Field &field, const world_new::view::RobotView thisRobot) {
    Target target;
    target.targetRobot = -1;
    target.targetPosition = calculateNewRobotPosition(field, thisRobot, thisRobot->getVel().toAngle());
    return target;
}


    MidFieldCoach::HarassType MidFieldCoach::getHarassType(const world_new::view::RobotView thisRobot, const world_new::view::RobotView opponent) {
        auto ball = world_new::World::instance()->getWorld()->getBall();
        BallPossession ballPossession;
        auto possession = ballPossession.getPossession();

        // Check if the opponent is left of us (is being offensive)
        if (opponent->getPos().x < thisRobot->getPos().x) {
            return getHarassTypeIfOpponentIsOnTheLeft(thisRobot, ball.value(), ballPossession, possession);

            // Else, the opponent is on our right (being defensive or about to offend)
        } else {
            if (possession == BallPossession::OURBALL) {
                return HARASS_OFFENSIVE;
            } else {
                return HARASS_DEFENSIVE;
            }
        }
    }

    MidFieldCoach::HarassType MidFieldCoach::getHarassTypeIfOpponentIsOnTheLeft(const world_new::view::RobotView thisRobot, const world_new::view::BallView ball,
                                                                                BallPossession &ballPossession, const BallPossession::Possession &possession) const {
        if (ball->getPos().x < thisRobot->getPos().x) {
            if (possession == BallPossession::OURBALL) {
                return STAND_FREE;
            } else {
                return HARASS_DEFENSIVE;
            }
            // Else, the ball is on our right
        } else {
            if (ballPossession.getPossession() == BallPossession::OURBALL) {
                return HARASS_OFFENSIVE;
            } else {
                return BLOCK_PASS;
            }
        }
    }

MidFieldCoach::Target MidFieldCoach::getBall(world_new::view::RobotView thisRobot, const world_new::view::RobotView opponent) {
    Target target;
    target.targetRobot = opponent->getId();
    target.targetPosition = world_new::World::instance()->getWorld()->getBall().value()->getPos();
    return target;
}

double MidFieldCoach::calculateStandingFreeScore(const Field &field, const Vector2 &position, const world_new::view::RobotView thisRobot) {
    auto world = world_new::World::instance()->getWorld().value();

    double passLineScore = CoachHeuristics::calculatePassLineScore(position, world);
    double distanceToUsScore = CoachHeuristics::calculateDistanceToClosestTeamMateScore(position, thisRobot->getId());
    double distanceToBallScore = CoachHeuristics::calculatePassDistanceToBallScore(field, position, world);

    return passLineScore + distanceToUsScore + distanceToBallScore;
}

Vector2 MidFieldCoach::calculateNewRobotPosition(const Field &field, const world_new::view::RobotView thisRobot, Angle targetAngle) {
    Vector2 bestPosition = targetPositions[thisRobot->getId()];
    double highestScore = calculateStandingFreeScore(field, bestPosition, thisRobot);

    Angle goldenAngle = 0.01;
    tick++;
    Angle thetaPlus = tick * tick * goldenAngle;
    Angle thetaMinus = -1 * tick * tick * goldenAngle;
    std::vector<Vector2> positions = {bestPosition + thetaPlus.toVector2(1.0 * GRID_SIZE),  bestPosition + thetaPlus.toVector2(2.0 * GRID_SIZE),
                                      bestPosition + thetaPlus.toVector2(4.0 * GRID_SIZE),  bestPosition + thetaMinus.toVector2(1.0 * GRID_SIZE),
                                      bestPosition + thetaMinus.toVector2(2.0 * GRID_SIZE), bestPosition + thetaMinus.toVector2(4.0 * GRID_SIZE)};

    for (const auto &position : positions) {
        if (!FieldComputations::pointIsInField(field, position, 0.20) || abs(position.x) > DISTANCE_FROM_MIDDLE_LINE) continue;
        double score = calculateStandingFreeScore(field, position, thisRobot);
        if (score > highestScore) {
            highestScore = score;
            bestPosition = position;
        }
    }

    targetPositions[thisRobot->getId()] = bestPosition;
    return bestPosition;
}

}  // namespace rtt::ai::coach
