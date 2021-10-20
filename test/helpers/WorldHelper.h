//
// Created by mrlukasbos on 28-1-19.
//

#ifndef ROBOTEAM_AI_WORLDHELPER_H
#define ROBOTEAM_AI_WORLDHELPER_H

#include <roboteam_proto/messages_robocup_ssl_geometry.pb.h>
#include <roboteam_proto/World.pb.h>
#include <roboteam_utils/Vector2.h>

namespace testhelpers {

class WorldHelper {
   public:
    static double getRandomValue(double min, double max);
    static rtt::Vector2 getRandomFieldPosition(proto::SSL_GeometryFieldSize field);
    static rtt::Vector2 getRandomVelocity();
    static bool allPositionsAreValid(const proto::World &worldMsg, bool withBall);
    static proto::WorldRobot generateRandomRobot(int id, proto::SSL_GeometryFieldSize field);
    static proto::WorldBall * generateRandomBall(proto::SSL_GeometryFieldSize field);
    static rtt::Vector2 getLocationRightBeforeRobot(proto::WorldRobot robot);
    static proto::WorldBall generateBallAtLocation(const rtt::Vector2 &loc);
    static google::protobuf::RepeatedPtrField<proto::WorldRobot> generateRandomRobots(int amount, const proto::SSL_GeometryFieldSize &field);
    static proto::World getWorldMsg(int amountUs, int amountThem, bool withBall, const proto::SSL_GeometryFieldSize &field);
};

}  // namespace testhelpers
#endif  // ROBOTEAM_AI_WORLDHELPER_H
