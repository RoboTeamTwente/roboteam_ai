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
    /**
     * Sets all the plays in the PlayChecker, takes ownership of plays
     * @param plays Moved vector of unique plays
     */
    void setPlays(std::vector<std::unique_ptr<Play>>& plays) noexcept;

    /**
     * Gets a vector of currently valid plays
     * @return map(plays, isValid)
     */
    [[nodiscard]] std::vector<Play*> getValidPlays() noexcept;

    /**
     * Sets this->world
     * @param world World to update against
     */
    void update(world_new::World* world) noexcept;

   private:
    /**
     * An array of all the plays
     */
    std::vector<std::unique_ptr<Play>>* allPlays{};

    /**
     * Current world, do not use before update()
     */
    world_new::World* world{};
};

}  // namespace rtt::ai::stp

#endif  // RTT_PLAYCHECKER_HPP
