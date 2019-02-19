//
// Created by mrlukasbos on 19-2-19.
//

#ifndef ROBOTEAM_AI_GAMEANALYZER_H
#define ROBOTEAM_AI_GAMEANALYZER_H

namespace rtt {
namespace ai {
namespace analysis {

class GameAnalyzer {
public:
    GameAnalyzer();
    double getBallPossessionEstimate(bool ourTeam);
private:
    double getBallSecurityForTeam(bool ourTeam = true);

};

}
}
}
#endif //ROBOTEAM_AI_GAMEANALYZER_H
