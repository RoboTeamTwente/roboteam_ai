//
// Created by baris on 21-2-19.
//

#ifndef ROBOTEAM_AI_GOBEHINDBALL_H
#define ROBOTEAM_AI_GOBEHINDBALL_H

#include "skills/Skill.h"
#include "world/FieldComputations.h"
#include "GoToPos.h"
#include <random>

namespace rtt::ai {

    class GoBehindBall : public GoToPos {

        private:
        std::mt19937 mt;
        std::uniform_real_distribution<double> randDistribution;
        int penaltyGenevaState = 1;
        enum RefType {
            penalty,
            freeKick,
            corner,
            shootOut
        };

        RefType refType;
        RefType stringToRefType(const std::string &string);
        const double errorMargin = Constants::ROBOT_RADIUS() + 0.05;

        Status penaltyUpdate(int genevaState);

        public:
        int chooseRandomGeneva(std::vector<std::pair<int, double>> genevaWithWeights);
        explicit GoBehindBall(string name, bt::Blackboard::Ptr blackboard);
        Status gtpUpdate() override;
        void gtpInitialize() override;
        void gtpTerminate(Status s) override;

    };

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_GOBEHINDBALL_H
