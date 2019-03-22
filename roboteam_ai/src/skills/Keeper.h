//
// Created by rolf on 10/12/18.
//

#ifndef ROBOTEAM_AI_KEEPER_H
#define ROBOTEAM_AI_KEEPER_H
#include "Skill.h"
#include "roboteam_utils/Arc.h"
#include "roboteam_utils/Math.h"
#include "roboteam_ai/src/control/PIDController.h"

namespace rtt {
namespace ai {
class Keeper : public Skill {

        const double KEEPER_POSDIF = 0.01;

    private:
        Arc blockCircle;
        Vector2 computeBlockPoint(Vector2 defendPos);
        Vector2 computeBlockPointWithAttacker(shared_ptr<roboteam_msgs::WorldRobot>);
        bool viewAtGoal(shared_ptr<roboteam_msgs::WorldRobot>);
        Vector2 goalPos;
        double goalwidth;
        void sendMoveCommand(Vector2 pos);
        void sendFineMoveCommand(Vector2 pos);
        void sendStopCommand();
        control::PIDController pid, finePid;

        control::PositionController goToPos;
    public:
        explicit Keeper(string name, bt::Blackboard::Ptr blackboard);
        Status onUpdate() override;
        void onInitialize() override;
        void onTerminate(Status s) override;
};
}
}

#endif //ROBOTEAM_AI_KEEPER_H
