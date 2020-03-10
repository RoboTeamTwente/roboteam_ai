//
// Created by rolf on 14/12/18.
//
// TODO: Test real robot rotation speeds.
// TODO: Make the robot automatically slow down/speed up if the ball is going to one end of the dribbler. Control?

#include <skills/DribbleRotate.h>
#include <coach/BallplacementCoach.h>

namespace rtt::ai {
DribbleRotate::DribbleRotate(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void DribbleRotate::checkProgression() {
    if (!robot->hasBall()) {
        currentProgression = FAIL;
        return;
    }
    if (abs(robot->get()->getAngle() - targetAngle) < 0.1 * M_PI) {
        currentProgression = SUCCESS;
        return;
    } else {
        currentProgression = ROTATING;
    }
}

void DribbleRotate::onInitialize() {
    if (properties->hasDouble("Angle")) {
        targetAngle = Angle(properties->getDouble("Angle"));
    } else if (properties->getBool("RotateToTheirGoal")) {
        Vector2 theirCentre = (*field).getTheirGoalCenter();
        targetAngle = (theirCentre - robot->get()->getPos()).toAngle();
    } else if (properties->getBool("BallPlacement")) {
        if (properties->getBool("BallPlacementForwards")) {
        }
        targetAngle = (Vector2(robot->get()->getPos()) - coach::g_ballPlacement.getBallPlacementPos()).toAngle();
    }
    if (!properties->hasDouble("Angle") && !properties->hasBool("RotateToTheirGoal") && !properties->hasBool("BallPlacement")) {
        std::cerr << " dribbleRotate Initialize -> No good angle set in properties" << std::endl;
        currentProgression = FAIL;
    }
    startAngle = robot->get()->getAngle();
    currentProgression = ROTATING;
    dir = control::ControlUtils::rotateDirection(startAngle, targetAngle);
    if (!world.ourRobotHasBall(robot->get()->getId(), Constants::MAX_BALL_RANGE())) {
        std::cout << "RobotPtr does not have ball in dribbleRotateInitialize" << std::endl;
        std::cout << "Distance" << (Vector2(robot->get()->getPos()) - Vector2(ball->get()->getPos())).length() - Constants::ROBOT_RADIUS()
                  << "Max distance:" << Constants::MAX_BALL_RANGE() << std::endl;
        currentProgression = FAIL;
        std::cout << robot->get()->getAngle().getAngle() << std::endl;
    } else {
        std::cout << "RobotPtr has ball in dribbleRotate Initialize" << std::endl;
    }
}

DribbleRotate::Status DribbleRotate::onUpdate() {
    checkProgression();
    switch (currentProgression) {
        case ROTATING:
            robot->getControllers().getBallHandlePosController()->getRobotCommand(robot->get()->getId(), ball->get()->getPos(), targetAngle);
            publishRobotCommand();
            return Status::Running;
        case SUCCESS:
            return Status::Success;
        case FAIL:
            return Status::Failure;
    }
    return Status::Failure;
}

void DribbleRotate::onTerminate(Status s) {
    command.set_dribbler(31);
    command.set_w(static_cast<float>(targetAngle));
    currentProgression = ROTATING;
    publishRobotCommand();
}

}  // namespace rtt::ai
