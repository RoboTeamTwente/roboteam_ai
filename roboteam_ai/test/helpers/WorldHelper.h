//
// Created by mrlukasbos on 28-1-19.
//

#ifndef ROBOTEAM_AI_WORLDHELPER_H
#define ROBOTEAM_AI_WORLDHELPER_H

#include <roboteam_utils/Vector2.h>
#include <roboteam_msgs/GeometryFieldSize.h>
#include <roboteam_msgs/World.h>

namespace testhelpers {

class WorldHelper {
    public:
        static void resetWorld();
        static double getRandomValue(double min, double max);
        static rtt::Vector2 getRandomFieldPosition(roboteam_msgs::GeometryFieldSize field);
        static rtt::Vector2 getRandomVelocity();
        static bool allPositionsAreValid(const roboteam_msgs::World &worldMsg, bool withBall);
        static roboteam_msgs::WorldRobot generateRandomRobot(int id, roboteam_msgs::GeometryFieldSize field);
        static roboteam_msgs::WorldBall generateRandomBall(roboteam_msgs::GeometryFieldSize field);
        static rtt::Vector2 getLocationRightBeforeRobot(roboteam_msgs::WorldRobot robot);
        static roboteam_msgs::WorldBall generateBallAtLocation(const rtt::Vector2 &loc);
        static std::vector<roboteam_msgs::WorldRobot> generateRandomRobots(int amount,
                const roboteam_msgs::GeometryFieldSize &field);
        static roboteam_msgs::World getWorldMsg(int amountUs, int amountThem, bool withBall,
                const roboteam_msgs::GeometryFieldSize &field);
        static std::pair<roboteam_msgs::World, int> getWorldMsgWhereRobotHasBall(int amountUs, int amountThem,
                bool weHaveBall, roboteam_msgs::GeometryFieldSize field);
};

}
#endif //ROBOTEAM_AI_WORLDHELPER_H
