//
// Created by robzelluf on 12/7/18.
//

#ifndef ROBOTEAM_AI_DEFENDONROBOT_H
#define ROBOTEAM_AI_DEFENDONROBOT_H

#include "Skill.h"
#include <boost/optional.hpp>
#include <roboteam_ai/src/conditions/HasBall.hpp>
#include <roboteam_ai/src/utilities/Coach.h>
#include <roboteam_ai/src/control/ControlGoToPos.h>

namespace rtt {
namespace ai {

class DefendOnRobot : public Skill {
    private:
        using status = bt::Node::Status;
        using goType = control::ControlGoToPos::GoToType;

        control::ControlGoToPos goToPos;
        double angleBetweenRobots;
        Vector2 newPosition;

        int amountOfCycles{};
    protected:
        enum Progression {
          IDLE, DONE, FAIL
        };
        Progression currentProgress;
    public:
        explicit DefendOnRobot(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
        void onInitialize() override;
        void onTerminate(Status s) override;
        Status onUpdate() override;
        Vector2 calculateLocation();
        int opponentWithBallID;
        int opponentToCoverID;
        std::shared_ptr<roboteam_msgs::WorldRobot> opponentWithBall;
        std::shared_ptr<roboteam_msgs::WorldRobot> opponentToCover;
};
} // ai
} // rtt


#endif //ROBOTEAM_AI_DEFENDONROBOT_H
