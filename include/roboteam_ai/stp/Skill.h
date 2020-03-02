//
// Created by john on 3/2/20.
//

#ifndef RTT_SKILL_H
#define RTT_SKILL_H

#include <bt/Node.h>
#include <world_new/views/RobotView.hpp>

#include "stp/StpInfo.h"

namespace rtt::ai::stp {
    class Skill : public bt::Node {
        using robot_view = world_new::view::RobotView;
    private:
        proto::RobotCommand command;
        robot_view robot;

    public:
        virtual void publishRobotCommand() noexcept;
        [[nodiscard]] virtual proto::RobotCommand rotateRobotCommand(proto::RobotCommand const& cmd) const noexcept;
        virtual void refreshRobotCommand() noexcept;
        virtual void limitRobotCommand() noexcept;

        virtual void terminate() noexcept = 0;
        virtual void onTerminate(Status s) noexcept;

        virtual void onUpdate() noexcept;

        virtual void onInitialize() noexcept;

        void refreshRobotPositionControllers() const noexcept;

        void update(SkillInfo const& info)
    };
}


#endif //RTT_SKILL_H
