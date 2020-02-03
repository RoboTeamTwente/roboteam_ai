#ifndef ROBOTEAM_AI_FORMATION_H
#define ROBOTEAM_AI_FORMATION_H

#include <control/numtrees/NumTreePosControl.h>
#include "gtest/gtest_prod.h"
#include "skills/Skill.h"

namespace rtt::ai {

    class Formation : public Skill {
        FRIEND_TEST(FormationTest, formation_test);

        public:
        explicit Formation(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
        void onInitialize() override;
        bt::Node::Status onUpdate() override;
        void onTerminate(bt::Node::Status) override;
        void terminate(bt::Node::Status) override;

        protected:
        // these two always need to be overridden
        virtual Vector2 getFormationPosition() = 0;
        virtual std::shared_ptr<std::vector<RobotPtr>> robotsInFormationPtr() = 0;

        Vector2 getOptimalPosition(int robotId, const std::vector<RobotPtr> &robotsInFormation, std::vector<Vector2> targetLocations);

        double errorMargin = 0.1;
        static std::vector<RobotPtr> robotsInFormation;
        static bool update;
        static int updateCount;
        int robotsInFormationMemory = 0;

        Vector2 targetLocation;

        void removeRobotFromFormation();
        void addRobotToFormation();
        bool robotIsInFormation();
        bool formationHasChanged();
        bool robotIsInPosition();
        virtual void updateFormation();
        void moveToTarget();
        virtual void setFinalAngle();
        bool updateCounter();
    };

}  // namespace rtt::ai
#endif  // ROBOTEAM_AI_FORMATION_H
