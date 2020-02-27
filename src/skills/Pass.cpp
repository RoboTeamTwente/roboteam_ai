//
// Created by robzelluf on 1/22/19.
//

#include <skills/Pass.h>
#include <coach/BallplacementCoach.h>
#include <control/NewControlUtils.h>

namespace rtt::ai {

Pass::Pass(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void Pass::onInitialize() {
    coach::g_pass.resetPass(-1);

    if (properties->hasString("passType")) {
        passType = stringToType(properties->getString("passType"));
    } else {
        passType = DEFAULT;
    }

    robotToPassToID = -1;
    passInitialized = false;
    hasShot = false;
    forcePass = false;
    fails = 0;
    if (properties->hasInt("failsUntilChip")) {
        maxTries = properties->getInt("failsUntilChip");
    } else {
        maxTries = -1;
    }
}

Pass::Status Pass::onUpdate() {
    bool closeToBall = (robot->get()->getPos() - ball->get()->getPos()).length() < CLOSE_ENOUGH_TO_BALL;

    // Only do this if not close to the ball and the pass is not yet initialized
    if (!closeToBall && !passInitialized) {
        RobotCommand robotCommand;

        robotToPassToID = coach::g_pass.getRobotBeingPassedTo();
        robotToPassTo = world_new::World::instance()->getWorld()->getRobotForId(robotToPassToID, true);

        if (!robotToPassTo || robotToPassToID == -1) {
            robotCommand = robot->getControllers().getNumTreePosController()->getRobotCommand(world, field, *robot, ball->get()->getPos());
        } else {
            robotCommand = robot->getControllers().getBallHandlePosController()->getRobotCommand(world, field, *robot, robotToPassTo->get()->getPos(),
                                                                                                 control::BallHandlePosControl::TravelStrategy::FORWARDS);
        }

        command.mutable_vel()->set_x(robotCommand.vel.x);
        command.mutable_vel()->set_y(robotCommand.vel.y);
        command.set_w(robotCommand.angle);
        publishRobotCommand();
        return Status::Running;

    } else {
        if (!passInitialized) {
            passInitialized = true;
            initiatePass();
        }

        robotToPassToID = coach::g_pass.getRobotBeingPassedTo();
        if (robotToPassToID == -1) {
            return Status::Failure;
        }

        robotToPassTo = world_new::World::instance()->getWorld()->getRobotForId(robotToPassToID, true);

        if (!coach::g_pass.validReceiver(*field, *robot, *robotToPassTo)) {
            return Status::Failure;
        }

        if (didShootProperly()) {
            coach::g_pass.setPassed(true);
            return Status::Success;
        }

        /// Check if:
        // Not already decided to chip
        // Not having already tried a shot
        // If this is both not the case, check if there's a clear line to the target
        // If not, either ++ fails or fail immediately
        if (!forcePass && !hasShot && !control::NewControlUtils::clearLine(ball->get()->getPos(), robotToPassTo->get()->getPos(), *world_new::World::instance()->getWorld(), 1)) {
            // If the passType is defensive, force to immediately chip as soon as the pass is blocked
            if (passType == DEFENSIVE || passType == FREEKICK) {
                forcePass = true;
            } else if (maxTries == -1) {
                return Status::Failure;
            } else {
                fails++;
                if (fails >= maxTries) {
                    forcePass = true;
                } else {
                    coach::g_pass.resetPass(robot->get()->getId());
                    initiatePass();
                }
            }
        }

        makeCommand();

        if ((command.kicker() == true || command.chipper() == true) && !hasShot) {
            hasShot = true;
        }
    }

    publishRobotCommand();
    return Status::Running;
}

void Pass::makeCommand() {
    RobotCommand shotdata;

    shotdata = robot->getControllers().getShotController()->getRobotCommand(*field, *robot, getKicker(), forcePass,
                                                                            control::PASS, control::HIGH,
                                                                            <#initializer#>, <#initializer#>);
    command = shotdata.makeROSCommand();
}

void Pass::onTerminate(Status s) {
    hasShot = false;
    passInitialized = false;
    if (!coach::g_pass.isPassed() || forcePass) {
        coach::g_pass.resetPass(robot->get()->getId());
    } else if (s == Status::Success) {
    }
}

Vector2 Pass::getKicker() {
    Vector2 distanceToKicker = {Constants::CENTRE_TO_FRONT(), 0};
    return robotToPassTo->get()->getPos() + distanceToKicker.rotate(robotToPassTo->get()->getAngle());
}

void Pass::initiatePass() { coach::g_pass.initiatePass(*field, robot->get()->getId()); }

bool Pass::didShootProperly() {
    bool ballIsMovingFast = Vector2(world->get()->getBall()->get()->getVelocity()).length() > 0.6;
    bool ballIsMovingToReceiver = true;  // control::ControlUtils::objectVelocityAimedToPoint(ball->pos, ball->vel,
    // robotToPassTo->pos, SUCCESSFUL_PASS_ANGLE);

    return (hasShot && ballIsMovingFast && ballIsMovingToReceiver);
}

Pass::PassType Pass::stringToType(const std::string &type) {
    if (type == "defensive") {
        return DEFENSIVE;
    } else if (type == "freeKick") {
        return FREEKICK;
    } else {
        return DEFAULT;
    }
}

}  // namespace rtt::ai
