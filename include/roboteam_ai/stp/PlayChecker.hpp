//
// Created by john on 3/9/20.
//

#ifndef RTT_PLAYCHECKER_HPP
#define RTT_PLAYCHECKER_HPP

#include <vector>

#include "Play.hpp"

namespace rtt::ai::stp {

    /**
     * Class that gets all the viable plays
     */
    class PlayChecker {
    public:

        std::vector<std::reference_wrapper<Play>> getValidPlays() noexcept;


    private:
        std::array<Play, 0> allPlays {};

        world_new::World* world;
        Play* currentValidPlay;
    };

} // namespace rt::ai::stp

#endif //RTT_PLAYCHECKER_HPP
