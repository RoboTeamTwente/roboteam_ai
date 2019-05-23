//
//

#include <gtest/gtest.h>
#include <roboteam_ai/test/helpers/FieldHelper.h>
#include <roboteam_ai/test/helpers/WorldHelper.h>
#include "roboteam_ai/src/utilities/Constants.h"
#include "roboteam_ai/src/coach/defence/DefencePositionCoach.h"
#include "roboteam_ai/src/coach/defence/DefenceDealer.h"
namespace w=rtt::ai::world;
namespace rtt{
namespace ai{
namespace coach{
w::Robot createRobot(const Vector2& pos,int id, bool ourTeam){
    w::Robot robot;
    robot.team=ourTeam? w::Robot::Team::us : w::Robot::Team::them;
    robot.pos = pos;
    robot.id=id;
    return robot;
}

TEST(defensive_coach,blockPoints){
    roboteam_msgs::GeometryFieldSize field = testhelpers::FieldHelper::generateField();
    w::field->set_field(field);
    w::world->updateWorld(testhelpers::WorldHelper::getWorldMsg(0,8,false,field));
    for (int j = 0; j < 500; ++ j) {
        w::world->updateWorld(testhelpers::WorldHelper::getWorldMsg(0,8,false,field));
        auto testWorld=w::world->getWorld();
        for (const auto& robot : w::world->getThem()){
            std::pair<Vector2,Vector2> goal=w::field->getGoalSides(true);
            Vector2 toBlockFrom=w::world->getRobotForId(robot.id,false)->pos;
            auto lineSegment=g_defensivePositionCoach.getBlockLineSegment(goal,toBlockFrom);
            auto blockPos=g_defensivePositionCoach.getBlockPoint(goal,toBlockFrom,Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS());
            testWorld.us.push_back(createRobot(blockPos,robot.id,true));
            auto parts=w::field->getVisiblePartsOfGoal(true,toBlockFrom,testWorld);
            EXPECT_EQ(parts.size(),0);

            if (lineSegment){
                if (w::field->pointIsInDefenceArea(blockPos,true,0.15)){
                    EXPECT_NE(blockPos,lineSegment->second);
                }
                else{
                    EXPECT_EQ(blockPos,lineSegment->second);
                }
                EXPECT_FALSE(w::field->pointIsInDefenceArea(lineSegment->first,true,Constants::ROBOT_RADIUS()));
                EXPECT_FALSE(w::field->pointIsInDefenceArea(lineSegment->second,true,Constants::ROBOT_RADIUS()));
            }
            testWorld.us.clear();

            // testing bot creation
            //TODO check defence area and other rule compliance
            if (lineSegment){
                DefenderBot bot1=g_defensivePositionCoach.createBlockBall(*lineSegment);
                EXPECT_LE(bot1.targetPos.x,g_defensivePositionCoach.maxX()+0.0000001); // aargh floating point errors
                auto passes=g_defensivePositionCoach.createPassesSortedByDanger(testWorld);
                for (auto& pass :passes) {
                    auto blockPosA = g_defensivePositionCoach.blockOnDefenseAreaLine(pass, testWorld);
                    if (blockPosA) {
                        if (g_defensivePositionCoach.validNewPosition(*blockPosA, testWorld)) {
                            DefenderBot bot2 = g_defensivePositionCoach.createBlockOnLine(pass, *blockPosA);
                            EXPECT_LE(bot2.targetPos.x,g_defensivePositionCoach.maxX());
                        }
                    }
                    //TODO test block on line
                    auto passBlock = g_defensivePositionCoach.pickNewPosition(pass, testWorld);
                    if (passBlock) {
                        DefenderBot bot3 = g_defensivePositionCoach.createBlockPass(pass, *passBlock);
                        // check if bot3 actually intercepts the pass.
                        EXPECT_LE(control::ControlUtils::distanceToLineWithEnds(bot3.targetPos, pass.startPos,
                                pass.endPos),1e-14); // give some room for floating point errors
                        EXPECT_LE(bot3.targetPos.x,g_defensivePositionCoach.maxX());

                    }
                    auto aggFac=g_defensivePositionCoach.pickNewPosition(*lineSegment,testWorld);
                    if (aggFac) {
                        DefenderBot bot4 = g_defensivePositionCoach.createBlockToGoal(pass,*aggFac,*lineSegment);
                        EXPECT_LE(bot4.targetPos.x,g_defensivePositionCoach.maxX());
                        // test if this blocks goal

                    }
                }
            }
        }
    }
}
TEST(defensive_coach,complete) {
    roboteam_msgs::GeometryFieldSize field = testhelpers::FieldHelper::generateField();
    w::field->set_field(field);
    w::world->updateWorld(testhelpers::WorldHelper::getWorldMsg(8, 8, false, field));
    for (int j = 0; j < 100; ++ j) {
        double r = (double) rand()/RAND_MAX;
        w::world->updateWorld(testhelpers::WorldHelper::getWorldMsg(8, 8, r < 0.5, field));
        for (const auto& robot: w::world->getUs()){
            g_DefenceDealer.addDefender(robot.id);
        }
        g_DefenceDealer.updateDefenderLocations();
        for (const auto& robot: w::world->getUs()){
            EXPECT_NE(g_DefenceDealer.getDefenderPosition(robot.id),nullptr);
        }
    }
}
}
}
}

