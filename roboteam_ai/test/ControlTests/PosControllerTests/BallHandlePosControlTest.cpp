//
// Created by mrlukasbos on 22-5-19.
//


#include <gtest/gtest.h>
#include <roboteam_ai/src/control/positionControllers/BallHandlePosControl.h>
#include <roboteam_ai/src/utilities/GameStateManager.hpp>
#include <roboteam_ai/test/helpers/FieldHelper.h>
#include <roboteam_ai/test/helpers/WorldHelper.h>

namespace rtt {
namespace ai {
namespace control {

TEST(BallHandlePosControlTest, it_sends_proper_commands) {
    BallHandlePosControl gtp;

    auto field = testhelpers::FieldHelper::generateField();
    auto world = testhelpers::WorldHelper::getWorldMsgWhereRobotHasBall(1, 0, true, field);
    world::world->updateWorld(world.first);
    auto robot = std::make_shared<world::Robot>(world::world->getUs().at(0));
    auto futureRobot =  world::world->getFutureRobot(robot, 0.04);

    gtp.updateVariables(robot, {2.0, 2.0}, M_PI);
    EXPECT_EQ(gtp.targetPos.x, 2.0);
    EXPECT_EQ(gtp.targetPos.y, 2.0);
    EXPECT_EQ(gtp.finalTargetPos.x, 2.0);
    EXPECT_EQ(gtp.finalTargetPos.y, 2.0);
    EXPECT_EQ(gtp.targetAngle, M_PI);
    EXPECT_EQ(gtp.finalTargetAngle, M_PI);

    // Start Travel backwards resets the approachPosition and the LockedAngle
    auto msg = gtp.B_startTravelBackwards();
    EXPECT_EQ(gtp.backwardsProgress, BallHandlePosControl::B_turning);
    EXPECT_EQ(gtp.B_approachPosition, Vector2());
    EXPECT_EQ(gtp.B_lockedAngle, Angle());
    EXPECT_EQ(gtp.targetPos, gtp.finalTargetPos);

    msg = gtp.B_startTravelBackwards();
    EXPECT_EQ(gtp.backwardsProgress, BallHandlePosControl::B_turning);
    EXPECT_EQ(gtp.B_approachPosition, Vector2());
    EXPECT_EQ(gtp.B_lockedAngle, Angle());
    EXPECT_EQ(gtp.targetPos, gtp.finalTargetPos);

    msg = gtp.F_sendApproachCommand();
    EXPECT_EQ(msg.dribbler, 0);
    EXPECT_FLOAT_EQ(msg.vel.length(), gtp.maxForwardsVelocity);
    EXPECT_EQ(msg.vel.angle(), (futureRobot->pos - world::world->getBall()->pos).angle());
    EXPECT_EQ(msg.angle, gtp.F_lockedAngle);

    msg = gtp.B_sendApproachCommand();
    EXPECT_EQ(msg.dribbler, 1);
    EXPECT_FLOAT_EQ(msg.vel.length(), gtp.maxBackwardsVelocity);
    EXPECT_EQ(msg.angle, gtp.B_lockedAngle);

    msg = gtp.F_sendDribbleForwardsCommand();
    EXPECT_EQ(msg.dribbler, 8);
    EXPECT_EQ(msg.angle, gtp.F_lockedAngle);

//    EXPECT_EQ(msg.vel.angle(), gtp.F_lockedAngle);
//    EXPECT_FLOAT_EQ(msg.vel.length(), gtp.maxForwardsVelocity);


    msg = gtp.F_startTravelForwards();
    EXPECT_EQ(gtp.F_lockedAngle, Angle());
    EXPECT_EQ(gtp.F_forwardsDribbleLine.first.x, 0);
    EXPECT_EQ(gtp.F_forwardsDribbleLine.first.y, 0);
    EXPECT_EQ(gtp.F_forwardsDribbleLine.second.x, 0);
    EXPECT_EQ(gtp.F_forwardsDribbleLine.second.y, 0);
    EXPECT_EQ(gtp.forwardsProgress, BallHandlePosControl::F_turning);


    msg = gtp.F_sendSuccessCommand();
    EXPECT_EQ(msg.angle, gtp.F_lockedAngle);
    EXPECT_EQ(msg.dribbler, 0);

    msg = gtp.B_sendDribblingCommand();
    EXPECT_EQ(msg.angle, gtp.B_lockedAngle);
    EXPECT_EQ(msg.dribbler, 16);
    EXPECT_EQ(msg.vel.x, 0);
    EXPECT_EQ(msg.vel.y, 0);

    msg = gtp.B_sendDribbleBackwardsCommand();
    EXPECT_EQ(msg.angle, gtp.B_lockedAngle);
    EXPECT_EQ(msg.dribbler, 24);

}


TEST(BallHandlePosControlTest, it_has_proper_prints) {
    BallHandlePosControl gtp;
    EXPECT_EQ(gtp.printForwardsProgress(BallHandlePosControl::F_start), "start");
    EXPECT_EQ(gtp.printForwardsProgress(BallHandlePosControl::F_turning), "turning");
    EXPECT_EQ(gtp.printForwardsProgress(BallHandlePosControl::F_approaching), "approaching");
    EXPECT_EQ(gtp.printForwardsProgress(BallHandlePosControl::F_dribbleForward), "dribble forwards");
    EXPECT_EQ(gtp.printForwardsProgress(BallHandlePosControl::F_success), "success");
    EXPECT_EQ(gtp.printForwardsProgress(BallHandlePosControl::F_fail), "fail");

    EXPECT_EQ(gtp.printBackwardsProgress(BallHandlePosControl::B_overshooting), "overshooting");
    EXPECT_EQ(gtp.printBackwardsProgress(BallHandlePosControl::B_dribbleBackwards), "dribbleBackwards");
    EXPECT_EQ(gtp.printBackwardsProgress(BallHandlePosControl::B_dribbling), "dribbling");
    EXPECT_EQ(gtp.printBackwardsProgress(BallHandlePosControl::B_success), "success");
    EXPECT_EQ(gtp.printBackwardsProgress(BallHandlePosControl::B_fail), "fail");
    EXPECT_EQ(gtp.printBackwardsProgress(BallHandlePosControl::B_start), "start");
    EXPECT_EQ(gtp.printBackwardsProgress(BallHandlePosControl::B_turning), "turning");
    EXPECT_EQ(gtp.printBackwardsProgress(BallHandlePosControl::B_approaching), "approaching");

    EXPECT_EQ(gtp.printTravelStrategy(BallHandlePosControl::forwards), "forwards");
    EXPECT_EQ(gtp.printTravelStrategy(BallHandlePosControl::backwards), "backwards");

    EXPECT_EQ(gtp.printRotateStrategy(BallHandlePosControl::rotateAroundBall), "aroundBall");
    EXPECT_EQ(gtp.printRotateStrategy(BallHandlePosControl::rotateAroundRobot), "aroundRobot");
}





} // control
} // ai
} // rtt
