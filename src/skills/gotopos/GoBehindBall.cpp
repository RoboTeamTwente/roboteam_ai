//
// Created by baris on 21-2-19.
//

#include "skills/gotopos/GoBehindBall.h"

namespace rtt::ai {

GoBehindBall::GoBehindBall(std::string name, bt::Blackboard::Ptr blackboard) : GoToPos(std::move(name), std::move(blackboard)) {
    std::random_device rd;
    mt = std::mt19937(rd());
    randDistribution = std::uniform_real_distribution<double>(0.0, 1.0);
}

Skill::Status GoBehindBall::gtpUpdate() {
    switch (refType) {
        case penalty: {
            return penaltyUpdate(penaltyGenevaState);
        }
        case shootOut: {
            return penaltyUpdate(3);
        }
        case freeKick: {
            auto ball = world->getBall();
            auto goal = (*field).getTheirGoalCenter();

            Vector2 v = goal - ball->get()->getPos();
            targetPos = ((v * -1.0).stretchToLength(rtt::ai::Constants::ROBOT_RADIUS() + 0.14)) + ball->get()->getPos();
            if ((targetPos - robot->get()->getPos()).length2() > errorMargin * errorMargin) {
                return Status::Running;
            } else {
                command.set_w((ball->get()->getPos() - robot->get()->getPos()).toAngle());
                publishRobotCommand();
                return Status::Success;
            }
        }
        case corner:
            break;
    }
    return Status::Failure;
}

void GoBehindBall::gtpInitialize() {
    goToType = stringToGoToType(properties->getString("goToType"));
    if (goToType == basic) {
        robot->getControllers().getBasicPosController()->setAvoidBallDistance(rtt::ai::Constants::ROBOT_RADIUS() + 0.10);
    }

    else if (goToType == numTree) {
        robot->getControllers().getNumTreePosController()->setAvoidBallDistance(rtt::ai::Constants::ROBOT_RADIUS() + 0.10);
    }
    if (properties->hasString("type")) {
        refType = stringToRefType(properties->getString("type"));
    }
    penaltyGenevaState = chooseRandomGeneva({{1, 0.5}, {5, 0.5}});
}

void GoBehindBall::gtpTerminate(Skill::Status s) {}

GoBehindBall::RefType GoBehindBall::stringToRefType(const std::string &string) {
    if (string == "penalty") {
        return penalty;
    } else if (string == "corner") {
        return corner;
    } else if (string == "freeKick") {
        return freeKick;
    } else if (string == "shootOut") {
        return shootOut;
    }
    std::cerr << "No string set for the RefType in GoBehindBall Skill!! using freeKick" << std::endl;
    return freeKick;
}
Skill::Status GoBehindBall::penaltyUpdate(int genevaState) {
    auto ball = world->getBall();
    auto goal = (*field).getTheirGoalCenter();

    Vector2 v = goal - ball->get()->getPos();
    targetPos = ((v * -1.0).stretchToLength(rtt::ai::Constants::ROBOT_RADIUS() + 0.2)) + ball->get()->getPos();
    command.set_geneva_state(genevaState);
    command.set_w(0);
    return (targetPos - robot->get()->getPos()).length2() > errorMargin * errorMargin ? Status::Running : Status::Success;
}

int GoBehindBall::chooseRandomGeneva(std::vector<std::pair<int, double>> genevaLikelyHood) {
    double totalLikelyHood = 0;
    std::vector<std::pair<int, double>> thresholds;
    for (auto states : genevaLikelyHood) {
        if (states.first > 0 && states.first < 6) {
            totalLikelyHood += states.second;
        }
    }
    double subsum = totalLikelyHood;
    for (auto states : genevaLikelyHood) {
        if (states.first > 0 && states.first < 6) {
            subsum -= states.second;
            thresholds.push_back(std::make_pair(states.first, subsum / totalLikelyHood));
        }
    }
    double randNum = randDistribution(mt);
    for (auto sub : thresholds) {
        if (randNum >= sub.second) {
            return sub.first;
        }
    }
    return thresholds[thresholds.size() - 1].first;  // failsave
}
}  // namespace rtt::ai
