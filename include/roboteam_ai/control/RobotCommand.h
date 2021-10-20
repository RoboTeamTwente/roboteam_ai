//
// Created by thijs on 29-4-19.
//

#ifndef ROBOTEAM_AI_ROBOTCOMMAND_H
#define ROBOTEAM_AI_ROBOTCOMMAND_H

#include <roboteam_proto/RobotCommand.pb.h>
#include "roboteam_utils/Angle.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

class RobotCommand {
   public:
    int id = -1;
    Vector2 pos = Vector2();
    Vector2 vel = Vector2();
    Angle angle = Angle();
    int dribbler = 0;
    bool kicker = false;
    double kickerVel = 0;
    bool kickerForced = false;
    bool chipper = false;
    double chipperVel = 0;
    bool chipperForced = false;

    RobotCommand() = default;
    RobotCommand(const RobotCommand &robotCommand) = default;
    constexpr RobotCommand(const Vector2 &pos, const Vector2 &vel, const Angle &angle) : pos(pos), vel(vel), angle(angle) {}

    const proto::RobotCommand makeROSCommand() const {
        proto::RobotCommand message;
        message.set_id(id);
        message.mutable_vel()->set_x(vel.x);
        message.mutable_vel()->set_y(vel.y);
        message.set_use_angle(true);
        message.set_w(angle);
        message.set_dribbler(dribbler);
        message.set_kicker(kicker);
        message.set_chipper(chipper);
        message.set_chip_kick_vel(std::max(kickerVel, chipperVel));
        message.set_chip_kick_forced(kickerForced || chipperForced);
        return message;
    }
};

}  // namespace rtt

#endif  // ROBOTEAM_AI_ROBOTCOMMAND_H
