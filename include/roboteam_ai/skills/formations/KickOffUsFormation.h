#ifndef ROBOTEAM_AI_KICKOFFFORMATION_H
#define ROBOTEAM_AI_KICKOFFFORMATION_H

#include "skills/formations/Formation.h"

namespace rtt::ai {

    class KickOffUsFormation : public Formation {
        public:
        explicit KickOffUsFormation(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

        private:
        Vector2 getFormationPosition() override;
        std::shared_ptr<std::vector<RobotPtr>> robotsInFormationPtr() override;
        static std::shared_ptr<std::vector<RobotPtr>> robotsInFormation;
    };

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_KICKOFFFORMATION_H
