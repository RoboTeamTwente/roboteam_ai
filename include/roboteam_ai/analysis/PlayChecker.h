//
// Created by jessevw on 04.12.19.
//

#ifndef RTT_PLAYCHECKER_H
#define RTT_PLAYCHECKER_H


namespace rtt::ai::analysis {
    class PlayChecker {
    public:
        PlayChecker();
        checkCurrentGameInvariants();

    private:
        /**
         * List of the invariants of the current strategy
         */
        std::vector<std::string> invariants;
        /**
         * Vector of all strategies (before pruning)
         */
        std::vector<std::string> allStrategies;
    }
}


#endif //RTT_PLAYCHECKER_H
