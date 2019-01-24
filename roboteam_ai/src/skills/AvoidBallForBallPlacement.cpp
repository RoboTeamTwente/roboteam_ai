//
// Created by mrlukasbos on 24-1-19.
//

#include "AvoidBallForBallPlacement.h"
#include "../utilities/Coach.h"
#include "../control/ControlUtils.h"

namespace rtt {
namespace ai {

AvoidBallForBallPlacement::AvoidBallForBallPlacement(std::string name, bt::Blackboard::Ptr blackboard)
: Skill(std::move(name), std::move(blackboard)) {}

void AvoidBallForBallPlacement::onInitialize() {
    ballPlacementTargetLocation = interface::InterfaceValues::getBallPlacementTarget();

    auto robotPos = rtt::Vector2(robot->pos);

    if (positionIsTooCloseToLine(robotPos)) {
        currentProgress = RUNNING;
    } else {
        currentProgress = DONE;
        return;
    }

    auto ballPos = rtt::Vector2(ball->pos);
    // get the target to move to which is the same vector from the ball
    auto projectionOnLinePoint = robotPos.project(ballPos, ballPlacementTargetLocation);
    auto lineDirection = ballPos - ballPlacementTargetLocation;
    auto inversedLineDirectionX = Vector2(-lineDirection.x, lineDirection.y).stretchToLength(.5);
    auto inversedLineDirectionY = Vector2(lineDirection.x, -lineDirection.y).stretchToLength(.5);


    auto point = projectionOnLinePoint + inversedLineDirectionX;
    if (Field::pointIsInField(point) && !positionIsTooCloseToLine(point)) {
        targetToMoveTo = point;
        return;
    }

    point = projectionOnLinePoint + inversedLineDirectionY;
    if (Field::pointIsInField(point) && !positionIsTooCloseToLine(point)) {
        targetToMoveTo = point;
        return;
    }

    // if we reach this point we are in a corner of the field.
    inversedLineDirectionX.stretchToLength(2);
    targetToMoveTo = control::ControlUtils::projectPositionToWithinField(projectionOnLinePoint + inversedLineDirectionX);
    if (positionIsTooCloseToLine(targetToMoveTo)) {
        inversedLineDirectionY.stretchToLength(2);
        targetToMoveTo = control::ControlUtils::projectPositionToWithinField(projectionOnLinePoint + inversedLineDirectionY);
    }
}

bt::Node::Status AvoidBallForBallPlacement::onUpdate() {
    if (currentProgress == DONE) {
      //  return bt::Node::Status::Success;
    }

    auto velocities = gtp.goToPos(robot, targetToMoveTo, control::GoToType::luTh);
    auto robotPos = rtt::Vector2(robot->pos);

    roboteam_msgs::RobotCommand cmd;
    cmd.id = robot->id;
    cmd.x_vel = static_cast<float>(velocities.x);
    cmd.y_vel = static_cast<float>(velocities.y);
    cmd.use_angle = 1;
    cmd.w = static_cast<float>((targetToMoveTo - robot->pos).angle());
    publishRobotCommand(cmd);

    if (robotPos.dist(targetToMoveTo) > constants::GOTOPOS_LUTH_ERROR_MARGIN) {
        // return bt::Node::Status::Success;
    }


    if (currentProgress == RUNNING) {
        return bt::Node::Status::Running;
    } else {
        return bt::Node::Status::Failure;
    }
}

bool AvoidBallForBallPlacement::positionIsTooCloseToLine(rtt::Vector2 position) {
    return control::ControlUtils::distanceToLineWithEnds(position, ball->pos, ballPlacementTargetLocation) < constants::ROBOT_RADIUS * 4;
}

} // ai
} // rtt