//
// Created by baris on 21-2-19.
//

#include "GoBehindBall.h"

namespace rtt {
namespace ai {

GoBehindBall::GoBehindBall(string name, bt::Blackboard::Ptr blackboard)
        :GoToPos(std::move(name), std::move(blackboard)) {
}

Skill::Status GoBehindBall::gtpUpdate() {

    switch (type) {
    case penalty: {
        auto ball = ai::world::world->getBall();
        auto goal = ai::world::field->get_their_goal_center();

        Vector2 v = goal - ball->pos;
        targetPos = ((v*- 1.0).stretchToLength(rtt::ai::Constants::ROBOT_RADIUS()+0.08)) + ball->pos;
       // command.geneva_state = 1;
        command.w = (rtt::ai::world::field->get_their_goal_center() - robot->pos).angle();
        return (targetPos - robot->pos).length2() > errorMargin * errorMargin ? Status::Running : Status::Success;
    }
    case freeKick: {
        auto ball = ai::world::world->getBall();
        auto goal = ai::world::field->get_their_goal_center();

        Vector2 v = goal - ball->pos;
        targetPos = ((v*- 1.0).stretchToLength(rtt::ai::Constants::ROBOT_RADIUS()+0.09)) + ball->pos;
        command.w = (rtt::ai::world::field->getPenaltyPoint(false) - robot->pos).angle();
        if ((targetPos - robot->pos).length2() > errorMargin * errorMargin) {
            return Status::Running;
        }
        else {
            publishRobotCommand();
            return Status::Success;
        }
    }
    case corner:break;
    case movingBall: {
        bool ballLayingStill = ball->vel.length() < Constants::BALL_STILL_VEL();
        bool robotHasBall = world::world->ourRobotHasBall(robot->id, true);
        if (ballLayingStill || robotHasBall) {
            return Status::Success;
        }

        Vector2 ballEndPos = ball->pos + ball->vel * Constants::MAX_INTERCEPT_TIME();

        // If already in ball line, go to the ball
        if (isOnBallLine()) {
            targetPos = ball->pos;

        // else, intercept the ball
        } else {
            double timeNeeded = ((ballEndPos - robot->pos).length())/(Constants::MAX_VEL()*0.4);
            targetPos = ball->pos + ball->vel*timeNeeded;
//            if (!world::field->pointIsInField(targetPos)) {
//                return Status::Failure;
//            }
        }

        command.w = (ball->pos - robot->pos).toAngle();
        return Status::Running;
    }

    }
    return Status::Failure;
}

void GoBehindBall::gtpInitialize() {
    posController->setAvoidBall(rtt::ai::Constants::DEFAULT_BALLCOLLISION_RADIUS());
    if (properties->hasString("type")) {
        type = stringToRefType(properties->getString("type"));
    }
}

void GoBehindBall::gtpTerminate(Skill::Status s) { }

GoBehindBall::RefType GoBehindBall::stringToRefType(const std::string &string) {
    if (string == "penalty") {
        return penalty;
    }
    else if (string == "corner") {
        return corner;
    }
    else if (string == "freeKick") {
        return freeKick;
    } else if (string == "movingBall") {
        return movingBall;
    }
    ROS_ERROR("No string set for the RefType in GoBehindBall Skill!! using freeKick");
    return freeKick;

}
bool GoBehindBall::isOnBallLine() {
    Angle ballAngle = ball->vel.toAngle();
    Angle robotToBallAngle = (robot->pos - ball->pos).toAngle();
    double angleDiff = abs((ballAngle - robotToBallAngle).getAngle());
    return angleDiff < ANGLE_MARGIN;
}

}
}
