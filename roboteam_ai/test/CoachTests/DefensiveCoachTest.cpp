//
//

#include <gtest/gtest.h>
#include <roboteam_ai/test/helpers/FieldHelper.h>
#include <roboteam_ai/test/helpers/WorldHelper.h>
#include "roboteam_ai/src/utilities/Constants.h"
#include "roboteam_ai/src/coach/defence/DefencePositionCoach.h"
namespace cc=rtt::ai::coach;
namespace w=rtt::ai::world;
namespace a= rtt::ai;
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

TEST(defensive_coach_test,blockPoints){
    roboteam_msgs::GeometryFieldSize field = testhelpers::FieldHelper::generateField();
    w::field->set_field(field);
    w::world->updateWorld(testhelpers::WorldHelper::getWorldMsg(0,8,false,field));
    for (int j = 0; j < 10000; ++ j) {
        w::world->updateWorld(testhelpers::WorldHelper::getWorldMsg(0,8,false,field));
        auto testWorld=w::world->getWorld();
        for (const auto& robot : w::world->getThem()){
            std::pair<Vector2,Vector2> goal=w::field->getGoalSides(true);
            Vector2 toBlockFrom=w::world->getRobotForId(robot.id,false)->pos;
            auto lineSegment=cc::g_defensivePositionCoach.getBlockLineSegment(goal,toBlockFrom);
            auto blockPos=cc::g_defensivePositionCoach.getBlockPoint(goal,toBlockFrom,a::Constants::ROBOT_RADIUS() + a::Constants::BALL_RADIUS());
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
                EXPECT_FALSE(w::field->pointIsInDefenceArea(lineSegment->first,true,0.15));
                EXPECT_FALSE(w::field->pointIsInDefenceArea(lineSegment->second,true,0.15));
            }

            testWorld.us.clear();
        }
    }
}
}
}
}

