//
// Created by rolf on 30-1-19.
//

#include <skills/gotopos/GoAroundPos.h>
#include <control/ControlUtils.h>

namespace rtt::ai {

GoAroundPos::GoAroundPos(std::string name, bt::Blackboard::Ptr blackboard) : GoToPos(std::move(name), std::move(blackboard)) {}

void GoAroundPos::gtpInitialize() {
    if (properties->hasBool("ball")) {
        if (ball) {
            ballIsTarget = true;
            targetPos = ball->get()->getPos();
        } else {
            std::cerr << "Get some balls" << std::endl;
        }
    } else {
        ballIsTarget = false;
        targetPos = properties->getVector2("targetPos");
    }

    if (properties->hasDouble("targetDir")) {
        endAngle = control::ControlUtils::constrainAngle(properties->getDouble("targetDir"));
    } else if (properties->getBool("towardsTheirGoal")) {
        endAngle = control::ControlUtils::constrainAngle((field->getTheirGoalCenter() - targetPos).angle());
    } else {
        endAngle = 0;
        std::cerr << "GoAroundPos update --> No target direction set! Defaulting to 0" << std::endl;
    }

    deltaPos = targetPos - robot->get()->getPos();
    startAngle = deltaPos.angle();
    rotateDir = control::ControlUtils::rotateDirection(startAngle, endAngle);
    if (ballIsTarget) {
        distanceFromPoint = BALL_DIST;
    } else {
        if (properties->hasDouble("RotatingDistance")) {
            distanceFromPoint = properties->getDouble("RotatingDistance");
        } else {
            std::cerr << "No rotating distance set! Defaulting to ball distance" << std::endl;
            distanceFromPoint = BALL_DIST;
        }
    }
    currentTick = 0;
    angleDif = control::ControlUtils::angleDifference(startAngle, endAngle);
    maxTick = floor(angleDif / SPEED * Constants::TICK_RATE());
    currentProgress = ROTATING;
}

GoAroundPos::Status GoAroundPos::gtpUpdate() {
    if (!robot) {
        std::cout << "RobotPtr not found ree: " << std::to_string(robot->get()->getId()).c_str() << std::endl;
        return Status::Failure;
    }
    if (ballIsTarget && !ball) {
        std::cout << "GoAroundPos update -> No ball found!" << std::endl;
        return Status::Failure;
    }
    if (ballIsTarget) {
        targetPos = ball->get()->getPos();
    }
    if (currentTick <= maxTick) {
        commandPos = targetPos + Vector2(distanceFromPoint, 0).rotate(startAngle + rotateDir * currentTick / maxTick * angleDif + M_PI);
    } else {
        commandPos = targetPos + Vector2(distanceFromPoint, 0).rotate(endAngle + M_PI);
    }

    deltaPos = targetPos - robot->get()->getPos();
    currentProgress = checkProgression();
    currentTick++;

    // Visualization
    std::vector<std::pair<rtt::Vector2, QColor>> displayColorData;
    displayColorData.emplace_back(commandPos, Qt::red);
    displayColorData.emplace_back(targetPos + Vector2(distanceFromPoint, 0).rotate(endAngle + M_PI), Qt::red);

    // interface::Input::setNumTreePoints(robot->id, displayColorData);

    switch (currentProgress) {
        case ROTATING: {
            sendRotateCommand();
            return Status::Running;
        }
        case STOPPING: {
            sendRotateCommand();
            return Status::Running;
        }
        case FAIL:
            return Status::Failure;
        case DONE:
            return Status::Success;
    }
    return Status::Failure;
}

void GoAroundPos::gtpTerminate(rtt::ai::Skill::Status s) { command.set_w((float)deltaPos.angle()); }

GoAroundPos::Progression GoAroundPos::checkProgression() {
    // Failure condition: If it goes outside of the margin during any phase but stopping (also helps against ball moving etc.)
    if (currentProgress != STOPPING) {
        if (!checkPosition()) {
            checkPosition();
            return FAIL;
        }
    }

    // Go to stopping if we are done rotating
    if (currentProgress == ROTATING) {
        if (currentTick > maxTick) {
            return STOPPING;
        } else {
            return ROTATING;
        }
    }

    if (currentProgress == STOPPING) {
        // Done when robot sufficiently close to desired end position and rotation.
        double angDif = control::ControlUtils::angleDifference(deltaPos.angle(), endAngle);
        double posDif = (commandPos - robot->get()->getPos()).length();
        if (posDif < POS_MARGIN && angDif < ANGLE_MARGIN) {
            return DONE;
        }
        // If RobotPtr takes too long to stop, fail
        if (currentTick > maxTick + MAX_STOP_TIME * Constants::TICK_RATE()) {
            return FAIL;
        } else {
            return STOPPING;
        }
    }
    return FAIL;
}

bool GoAroundPos::checkPosition() {
    double currentAngle = deltaPos.angle();
    double totalSum = control::ControlUtils::angleDifference(startAngle, currentAngle) + control::ControlUtils::angleDifference(currentAngle, endAngle);
    if (totalSum > angleDif + 0.1 * M_PI * 2) {
        return false;
    }
    return ((deltaPos.length() <= (distanceFromPoint + MAX_DIST_DEVIATION)) && deltaPos.length() > distanceFromPoint - MAX_DIST_DEVIATION);
}

void GoAroundPos::sendRotateCommand() {
    Vector2 deltaCommandPos = (commandPos - robot->get()->getPos());
    command.set_dribbler(0);
    command.mutable_vel()->set_x(static_cast<float>(deltaCommandPos.x));
    command.mutable_vel()->set_y(static_cast<float>(deltaCommandPos.y));
    command.set_w((float)deltaPos.angle());
    publishRobotCommand();
}
}  // namespace rtt::ai