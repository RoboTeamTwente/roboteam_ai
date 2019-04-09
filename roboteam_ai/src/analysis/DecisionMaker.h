//
// Created by mrlukasbos on 9-4-19.
//

#ifndef ROBOTEAM_AI_DECISIONMAKER_H
#define ROBOTEAM_AI_DECISIONMAKER_H

// define some play styles to influence our decision making
enum playStyleScore {
    DEFEND                          = 0,
    DEFEND_WITH_ALL_MIDFIELDERS     = 1,
    DEFEND_WITH_SOME_MIDFIELDERS    = 2,
    UNSURE_DEFENSIVE                = 3,
    UNSURE_OFFENSIVE                = 4,
    ATTACK_WITH_SOME_MIDFIELDERS    = 5,
    ATTACK_WITH_ALL_MIDFIELDERS     = 6,
    MAKE_THEM_PAY                   = 7,
};


struct playStyle {
    playStyleScore score;
    int amountOfDefenders;
    int amountOfMidfielders;
    int amountOfAttackers;

    playStyle(playStyleScore score, int def, int mid, int att)
        : score(score), amountOfDefenders(def), amountOfMidfielders(mid), amountOfAttackers(att) {};

    playStyleScore getScore() const {
        return score;
    }

    int getAmountOfDefenders() const {
        return amountOfDefenders;
    }

    int getAmountOfMidfielders() const {
        return amountOfMidfielders;
    }

    int getAmountOfAttackers() const {
        return amountOfAttackers;
    }
};


class DecisionMaker {
public:
    explicit DecisionMaker() = default;
    playStyle getRecommendedPlayStyle();

private:
    playStyle currentPlayStyle;

};


#endif //ROBOTEAM_AI_DECISIONMAKER_H
