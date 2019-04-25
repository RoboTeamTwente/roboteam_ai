//
// Created by thijs on 18-12-18.
//

#ifndef ROBOTEAM_AI_BALLHANDLEPOSCONTROL_H
#define ROBOTEAM_AI_BALLHANDLEPOSCONTROL_H

#include <roboteam_utils/Vector2.h>
#include "NumTreePosControl.h"

namespace rtt {
namespace ai {
namespace control {

class BallHandlePosControl : public PosController {
    private:
        std::shared_ptr<NumTreePosControl> numTreePosController;

        void checkInterfacePID() override;
    public:
        BallHandlePosControl() = default;
        explicit BallHandlePosControl(bool canMoveInDefenseArea);

        PosVelAngle getPosVelAngle(const RobotPtr &robot, const Vector2 &target, const Angle &targetAngle) override;
        PosVelAngle getPosVelAngle(const RobotPtr &robot, const Vector2 &target) override;
};

} //control
} //ai
} //rtt

#endif //ROBOTEAM_AI_BALLHANDLEPOSCONTROL_H
