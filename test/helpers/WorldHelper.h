//
// Created by mrlukasbos on 28-1-19.
//

#ifndef ROBOTEAM_AI_WORLDHELPER_H
#define ROBOTEAM_AI_WORLDHELPER_H

#include <roboteam_utils/Vector2.h>
#include <GeometryFieldSize.pb.h>
#include <World.pb.h>

namespace testhelpers {

class WorldHelper {
    public:
        static double getRandomValue(double min, double max);
        static rtt::Vector2 getRandomFieldPosition(roboteam_proto::GeometryFieldSize field);
        static rtt::Vector2 getRandomVelocity();
        static bool allPositionsAreValid(const roboteam_proto::World &worldMsg, bool withBall);
        static roboteam_proto::WorldRobot generateRandomRobot(int id, roboteam_proto::GeometryFieldSize field);
        static roboteam_proto::WorldBall generateRandomBall(roboteam_proto::GeometryFieldSize field);
        static rtt::Vector2 getLocationRightBeforeRobot(roboteam_proto::WorldRobot robot);
        static roboteam_proto::WorldBall generateBallAtLocation(const rtt::Vector2 &loc);
        static google::protobuf::RepeatedPtrField<roboteam_proto::WorldRobot> generateRandomRobots(int amount,
                const roboteam_proto::GeometryFieldSize &field);
        static roboteam_proto::World getWorldMsg(int amountUs, int amountThem, bool withBall,
                const roboteam_proto::GeometryFieldSize &field);
        static std::pair<roboteam_proto::World, int> getWorldMsgWhereRobotHasBall(int amountUs, int amountThem,
                bool weHaveBall, roboteam_proto::GeometryFieldSize field);
};

}
#endif //ROBOTEAM_AI_WORLDHELPER_H
