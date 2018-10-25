//
// Created by thijs on 24-10-18.
//

#ifndef ROBOTEAM_AI_ROTATEAROUNDPOINT_H
#define ROBOTEAM_AI_ROTATEAROUNDPOINT_H

#include "../utilities/World.h"
#include <boost/optional.hpp>
#include "Skill.h"
#include "roboteam_msgs/Vector2f.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Math.h"

#define PI 3.14159

namespace rtt {
namespace ai {

        class Rotate : public Skill {
        private:
            int amountOfCycles{};
        protected:
            virtual void sendRotationCommand(double angularVelocity);
        public:
            Status Update() override;
            void Initialize() override;
        };

} // ai
} // rtt

#endif //ROBOTEAM_AI_ROTATEAROUNDPOINT_H
