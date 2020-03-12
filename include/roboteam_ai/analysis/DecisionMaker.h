//
// Created by mrlukasbos on 9-4-19.
//

#ifndef ROBOTEAM_AI_DECISIONMAKER_H
#define ROBOTEAM_AI_DECISIONMAKER_H

#include "AnalysisReport.h"
// define some play styles to influence our decision making
enum playStyleScore : short { DEFENSIVE, NEUTRAL, OFFENSIVE };

namespace rtt::ai::analysis {

struct PlayStyle {
    int amountOfDefenders;
    int amountOfMidfielders;
    int amountOfAttackers;

    PlayStyle() = default;

    PlayStyle(int def, int mid, int att) : amountOfDefenders(def), amountOfMidfielders(mid), amountOfAttackers(att){};
};

class DecisionMaker {
   public:
    explicit DecisionMaker() = default;
    PlayStyle getRecommendedPlayStyle(BallPossession possession, uint8_t amountOfRobots);
};

}  // namespace rtt::ai::analysis

#endif  // ROBOTEAM_AI_DECISIONMAKER_H
