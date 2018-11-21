//
// Created by mrlukasbos on 23-10-18.
//

#ifndef ROBOTEAM_AI_SHOOT_H
#define ROBOTEAM_AI_SHOOT_H

#include "Skill.h"
#include <boost/optional.hpp>
#include "../utilities/World.h"

namespace rtt {
namespace ai {

class Kick : public Skill {
    private:
        using status = bt::Node::Status;
        int amountOfCycles{};
    protected:
        virtual void sendKickCommand(double kickVel);
        enum Progression {
          KICKING, DONE, FAIL, INVALID
        };
        Progression currentProgress;

        roboteam_msgs::WorldRobot robot;

    public:
        explicit Kick(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);

        Status update() override;

        void initialize() override;

        void terminate(status s) override;
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_SHOOT_H
