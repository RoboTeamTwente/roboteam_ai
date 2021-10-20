//
// Created by john on 3/9/20.
//

#include "stp/PlayDecider.hpp"

namespace rtt::ai::stp {

    bool PlayDecider::interfacePlayChanged = false;

    Play *PlayDecider::decideBestPlay(const std::vector<Play *>& plays, PlayEvaluator& playEvaluator) noexcept {
        if (interfacePlay) {
            return interfacePlay;
        }

        std::vector<std::pair<Play*, uint8_t>> playsWithScores;
        playsWithScores.reserve(plays.size());

        for (auto play : plays) {
            playsWithScores.emplace_back(std::make_pair(play, play->score(playEvaluator)));
        }

        std::sort(playsWithScores.begin(), playsWithScores.end(), [](auto &a, auto &b) {
            std::cout << a.first << ": " << a.second << std::endl;
            return a.second > b.second;
        });
        return playsWithScores.begin()->first;
    }

    // This is only used by the interface to force new plays
    void PlayDecider::lockInterfacePlay(Play *play) {
        PlayDecider::interfacePlayChanged = true;
        interfacePlay = play;
    }
}  // namespace rtt::ai::stp