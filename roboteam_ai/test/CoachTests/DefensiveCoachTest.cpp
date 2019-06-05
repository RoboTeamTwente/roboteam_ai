//
//

#include <gtest/gtest.h>
#include <roboteam_ai/test/helpers/FieldHelper.h>
#include <roboteam_ai/test/helpers/WorldHelper.h>
#include "roboteam_ai/src/utilities/Constants.h"
#include "roboteam_ai/src/coach/defence/DefencePositionCoach.h"
#include "roboteam_ai/src/coach/defence/DefenceDealer.h"
#include "roboteam_ai/src/control/ControlUtils.h"

namespace w=rtt::ai::world;
namespace rtt {
namespace ai {
namespace coach {
w::Robot::RobotPtr createRobot(const Vector2 &pos, int id, bool ourTeam) {
    w::Robot::RobotPtr robot = std::make_shared<world::Robot>();
    robot->team = ourTeam ? w::Robot::Team::us : w::Robot::Team::them;
    robot->pos = pos;
    robot->id = id;
    return robot;
}

TEST(defensive_coach, blockPoints) {
    roboteam_msgs::GeometryFieldSize field = testhelpers::FieldHelper::generateField();
    w::field->set_field(field);
    w::world->updateWorld(testhelpers::WorldHelper::getWorldMsg(0, 8, false, field));
    for (int j = 0; j < 500; ++ j) {
        w::world->updateWorld(testhelpers::WorldHelper::getWorldMsg(0, 8, false, field));
        g_defensivePositionCoach.simulatedWorld = g_defensivePositionCoach.setupSimulatedWorld();
        auto testWorld = w::world->getWorld();
        for (const auto &robot : w::world->getThem()) {
            std::pair<Vector2, Vector2> goal = w::field->getGoalSides(true);
            Vector2 toBlockFrom = w::world->getRobotForId(robot->id, false)->pos;
            auto lineSegment = g_defensivePositionCoach.getBlockLineSegment(goal, toBlockFrom);
            auto blockPos = g_defensivePositionCoach.getBlockPoint(goal, toBlockFrom,
                    Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
            testWorld.us.push_back(createRobot(blockPos, robot->id, true));
            auto parts = w::field->getVisiblePartsOfGoal(true, toBlockFrom, testWorld);
            EXPECT_EQ(parts.size(), 0);

            if (lineSegment) {
                if (w::field->pointIsInDefenceArea(blockPos, true, 0.15) || ! w::field->pointIsInField(blockPos)) {
                    EXPECT_NE(blockPos, lineSegment->second);
                }
                else {
                    EXPECT_EQ(blockPos, lineSegment->second);
                }
                EXPECT_FALSE(w::field->pointIsInDefenceArea(lineSegment->first, true, Constants::ROBOT_RADIUS()));
                EXPECT_FALSE(w::field->pointIsInDefenceArea(lineSegment->second, true, Constants::ROBOT_RADIUS()));
            }
            testWorld.us.clear();

            // testing bot creation and intended behavior
            if (lineSegment) {
                auto bot1 = g_defensivePositionCoach.blockMostDangerousPos();
                if (bot1) {
                    EXPECT_LE(bot1->targetPos.x,
                            g_defensivePositionCoach.maxX() + 1e-14); // aargh floating point errors
                    EXPECT_FALSE(w::field->pointIsInDefenceArea(bot1->targetPos, true, Constants::ROBOT_RADIUS()));
                    Vector2 dangerPos = g_defensivePositionCoach.getMostDangerousPos(testWorld);
                    double percentageBefore = w::field->getPercentageOfGoalVisibleFromPoint(true, dangerPos, testWorld);
                    testWorld.us.push_back(bot1->toRobot());
                    double percentageAfter = w::field->getPercentageOfGoalVisibleFromPoint(true, dangerPos, testWorld);
                    if (percentageBefore != 0) {
                        EXPECT_LT(percentageAfter, percentageBefore);
                    }
                    testWorld.us.clear();
                }
                auto passes = g_defensivePositionCoach.createPassesSortedByDanger(testWorld);
                for (auto &pass :passes) {
                    auto blockPosA = g_defensivePositionCoach.blockOnDefenseAreaLine(pass, testWorld);
                    if (blockPosA) {
                        if (g_defensivePositionCoach.validNewPosition(*blockPosA, testWorld)) {
                            DefenderBot bot2 = g_defensivePositionCoach.createBlockOnLine(pass, *blockPosA);
                            EXPECT_LE(bot2.targetPos.x, g_defensivePositionCoach.maxX());
                            EXPECT_FALSE(
                                    w::field->pointIsInDefenceArea(bot2.targetPos, true, Constants::ROBOT_RADIUS()));
                            // check if they actually covered the goal
                            double percentageBefore = w::field->getPercentageOfGoalVisibleFromPoint(true, pass.endPos,
                                    testWorld);
                            testWorld.us.push_back(bot2.toRobot());
                            double percentageAfter = w::field->getPercentageOfGoalVisibleFromPoint(true, pass.endPos,
                                    testWorld);
                            if (percentageBefore != 0) {
                                EXPECT_LT(percentageAfter, percentageBefore);
                            }
                            else {
                                EXPECT_EQ(percentageAfter, 0);
                            }
                            testWorld.us.clear();
                        }
                    }
                    auto passBlock = g_defensivePositionCoach.pickNewPosition(pass, testWorld);
                    if (passBlock) {
                        DefenderBot bot3 = g_defensivePositionCoach.createBlockPass(pass, *passBlock);
                        // check if bot3 actually intercepts the pass.
                        EXPECT_LE(control::ControlUtils::distanceToLineWithEnds(bot3.targetPos, pass.startPos,
                                pass.endPos),
                                1e-14); // give some room for floating point errors. This should be more than enough
                        EXPECT_LE(bot3.targetPos.x, g_defensivePositionCoach.maxX());
                        EXPECT_FALSE(w::field->pointIsInDefenceArea(bot3.targetPos, true, Constants::ROBOT_RADIUS()));

                    }
                    auto blockLine = g_defensivePositionCoach.blockToGoalLine(pass, testWorld);
                    if (blockLine) {
                        auto aggFac = g_defensivePositionCoach.pickNewPosition(*blockLine, testWorld);
                        if (aggFac) {
                            DefenderBot bot4 = g_defensivePositionCoach.createBlockToGoal(pass, *aggFac, *blockLine);
                            EXPECT_LE(bot4.targetPos.x, g_defensivePositionCoach.maxX());
                            EXPECT_FALSE(
                                    w::field->pointIsInDefenceArea(bot4.targetPos, true, Constants::ROBOT_RADIUS()));
                            // check if they actually covered the goal
                            double percentageBefore = w::field->getPercentageOfGoalVisibleFromPoint(true, pass.endPos,
                                    testWorld);
                            auto partsBefore = w::field->getVisiblePartsOfGoal(true, pass.endPos, testWorld);
                            testWorld.us.push_back(bot4.toRobot());
                            auto partsAfter = w::field->getVisiblePartsOfGoal(true, pass.endPos, testWorld);
                            double percentageAfter = w::field->getPercentageOfGoalVisibleFromPoint(true, pass.endPos,
                                    testWorld);
                            if (percentageBefore != 0) {
                                EXPECT_LT(percentageAfter, percentageBefore);
                            }
                            else {
                                EXPECT_EQ(percentageAfter, 0);
                            }
                            if (! partsBefore.empty()) {
                                EXPECT_LT(partsAfter.size(), partsBefore.size());
                            }
                            else {
                                EXPECT_TRUE(partsAfter.empty());
                            }
                            testWorld.us.clear();
                        }
                    }
                    auto generalBot = g_defensivePositionCoach.blockPass(pass);
                    if (generalBot) {
                        EXPECT_LE(generalBot->targetPos.x, g_defensivePositionCoach.maxX());
                        EXPECT_FALSE(
                                w::field->pointIsInDefenceArea(generalBot->targetPos, true, Constants::ROBOT_RADIUS()));
                    }
                }
            }
        }
    }
}
// uncomment this once defensivepositions algorithm guarantees to give back positions. Currently will fail but is not to big of an issue
TEST(defensive_coach, complete) {
    roboteam_msgs::GeometryFieldSize field = testhelpers::FieldHelper::generateField();
    w::field->set_field(field);
    w::world->updateWorld(testhelpers::WorldHelper::getWorldMsg(8, 8, false, field));
    for (int j = 0; j < 1000; ++ j) {
        double r = (double) rand()/RAND_MAX;
        w::world->updateWorld(testhelpers::WorldHelper::getWorldMsg(8, 8, r < 0.5, field));
        for (const auto &robot: w::world->getUs()) {
            g_DefenceDealer.addDefender(robot->id);
        }
        g_DefenceDealer.updateDefenderLocations();
        for (const auto &robot: w::world->getUs()) {
            EXPECT_NE(g_DefenceDealer.getDefenderPosition(robot->id), nullptr);
        }
    }
}

}
}
}

