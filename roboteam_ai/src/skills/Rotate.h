//
// Created by thijs on 24-10-18.
//

#ifndef ROBOTEAM_AI_ROTATEAROUNDPOINT_H
#define ROBOTEAM_AI_ROTATEAROUNDPOINT_H

#include "Skill.h"
#include <boost/optional.hpp>
#include "roboteam_utils/Vector2.h"
#include "../utilities/World.h"
#include "../utilities/Field.h"



namespace rtt {
namespace ai {

        class Rotate : public Skill {
        public:

            using worldBall = roboteam_msgs::WorldBall;
            using worldRobot = roboteam_msgs::WorldRobot;
            explicit Rotate(string name, bt::Blackboard::Ptr blackboard);

            Status Update() override;

            void Initialize() override;

        private:
            enum Progression {
                ROTATING, DONE, FAIL
            };
            Progression currentProgress;
            double targetRotation;
            int targetObject;
            bool rotateToObject;
        protected:
            virtual void sendRotationCommand(double angularVelocity);
        };

} // ai
} // rtt

#endif //ROBOTEAM_AI_ROTATEAROUNDPOINT_H
