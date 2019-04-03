//
//

#include <gtest/gtest.h>
#include "roboteam_ai/src/coach/DefensiveCoach.h"
#include "roboteam_ai/src/utilities/Constants.h"
namespace cc=rtt::ai::coach;
namespace w=rtt::ai;

TEST(DefensiveCoachTest,it_calculates_blockPoint) {
    rtt::Vector2 startPos(0, 3);
    rtt::Vector2 endPos(3, 0);
    rtt::Vector2 pointPos(0, 0);
    auto segment = std::make_pair(startPos, endPos);
    double colllisionRadius=w::Constants::ROBOT_RADIUS()+w::Constants::BALL_RADIUS();
    auto blockPoint = cc::g_defensiveCoach.getBlockPoint(segment, pointPos,colllisionRadius);
    ASSERT_FLOAT_EQ(blockPoint.angle(), M_PI_4);
    ASSERT_FLOAT_EQ(blockPoint.x, colllisionRadius);
    ASSERT_FLOAT_EQ(blockPoint.y, colllisionRadius);
    rtt::Vector2 middlePoint(3,3),checkPoint(3,1.5);
    auto blockPoint2 = cc::g_defensiveCoach.getBlockPoint(std::make_pair(middlePoint,endPos), pointPos,colllisionRadius);
    ASSERT_FLOAT_EQ(blockPoint2.angle(),checkPoint.angle());
}
