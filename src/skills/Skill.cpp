#include <control/ControlUtils.h>
#include <include/roboteam_ai/utilities/Settings.h>
#include <skills/Skill.h>
#include <utilities/Constants.h>
#include <utilities/RobotDealer.h>
#include <cmath>
#include <utilities/GameStateManager.hpp>

namespace rtt::ai {

Skill::Skill(std::string name, bt::Blackboard::Ptr blackboard) : bt::Leaf(std::move(name), std::move(blackboard)) {}

void Skill::publishRobotCommand() {
    if (!robot.has_value()) {
        std::cout << "[Skill::publishRobotCommand] Prevented sending command to non-existing robot" << std::endl;
        return;
    }

    if (!SETTINGS.isLeft()) command = rotateRobotCommand(command);

    limitRobotCommand();

    command.set_id((*robot)->getId());

    io::io.publishRobotCommand(command);

    resetRobotCommand();
}

std::string Skill::node_name() { return name; }

void Skill::initialize() {
    robot = getRobotFromProperties(properties);
    ball = world->getBall();
    if (!robot.has_value()) {
        std::cout << "[Skill::initialize] Warning. Trying to initialize Skill without the robot present" << std::endl;
        return;
    }
    if (!ball) {
        std::cout << "[Skill::initialize] Warning. Trying to initialize Skill without a ball present" << std::endl;
        return;
    }
    resetRobotCommand();
    onInitialize();
}

Skill::Status Skill::update() {
    std::string roleName = properties->getString("ROLE");
    robotId = rtt::ai::robotDealer::RobotDealer::findRobotForRole(roleName);
    updateRobot();
    ball = world->getBall();

    // Fail if the robot isn't present
    if (!robot.has_value()) return Status::Failure;

    // Wait if the ball isn't present (Emiel : Why? What if it is a driving skill)
    if (!ball.has_value()) return Status::Waiting;

    return onUpdate();
}

void Skill::terminate(Status s) {
    if (!init) return;
    init = false;
    if (!robot.has_value()) return;
    if (!ball) return;
    refreshRobotPositionControllers();
    resetRobotCommand();
    onTerminate(s);
}

proto::RobotCommand Skill::rotateRobotCommand(proto::RobotCommand &cmd) {
    proto::RobotCommand output = cmd;
    output.mutable_vel()->set_x(-cmd.vel().x());
    output.mutable_vel()->set_y(-cmd.vel().y());
    output.set_w(static_cast<float>(control::ControlUtils::constrainAngle(cmd.w() + M_PI)));
    return output;
}

void Skill::resetRobotCommand() {
    proto::RobotCommand emptyCmd;
    emptyCmd.set_use_angle(true);
    emptyCmd.set_id(robot.has_value() ? robot.value()->getId() : -1);
    emptyCmd.set_geneva_state(0);
    command.CopyFrom(emptyCmd);
}

/// Velocity and acceleration limiters used on command
void Skill::limitRobotCommand() {
    bool isDefendPenaltyState = rtt::ai::GameStateManager::getCurrentGameState().keeperStrategyName == "keeper_penalty_defend_tactic";
    bool isKeeper = command.id() == robotDealer::RobotDealer::getKeeperID();

    auto limitedVel = Vector2(command.vel().x(), command.vel().y());
    limitedVel = control::ControlUtils::velocityLimiter(limitedVel);

    if (!(isDefendPenaltyState && isKeeper)) {
        //limitedVel = control::ControlUtils::accelerationLimiter(limitedVel, previousRobotVelocity[robot->get()->getId()], command.w());
        //std::cerr << previousRobotVelocity[robot->get()->getId()] << '\n';
    }

    previousRobotVelocity[robot->get()->getId()] = robot->get()->getVel();
    if (std::isnan(limitedVel.x) || std::isnan(limitedVel.y)) {
        std::cout << "ERROR: ROBOT WILL HAVE NAN~!?!?!KLJ#Q@?LK@ " << node_name().c_str() << "!"
                  << "  robot  " << robot->get()->getId() << std::endl;
        previousRobotVelocity[robot->get()->getId()] = robot->get()->getVel();
    }
    command.mutable_vel()->set_x(limitedVel.x);
    command.mutable_vel()->set_y(limitedVel.y);
}

void Skill::refreshRobotPositionControllers() {
    robot->get()->resetNumTreePosControl();
    robot->get()->resetShotController();
    robot->get()->resetBallHandlePosControl();
    robot->get()->resetBasicPosControl();
}

}  // namespace rtt::ai
