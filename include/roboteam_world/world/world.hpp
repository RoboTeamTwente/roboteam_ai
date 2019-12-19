//
// Created by john on 12/16/19.
//

#ifndef RTT_WORLD_HPP
#define RTT_WORLD_HPP
#include <vector>
#include "world_data.hpp"


namespace rtt::world {

    /**
     * World operates under hte assumption of immutable states.
     * As soon as a world needs to be updates
     * the current wold is pushed to the history,
     * and currentWorld is re-constructed
     */
    class World {
    public:
        constexpr static size_t HISTORY_SIZE = 20;

        WorldData const& setWorld(WorldData& currentWorld) noexcept;
        [[nodiscard]] WorldData const& getWorld() const noexcept;
        [[nodiscard]] WorldData const& getHistoryWorld(size_t ticksAgo) const noexcept;

    private:
        void toHistory(WorldData& world) noexcept;

        std::vector<WorldData> history{ HISTORY_SIZE };
        size_t currentIndex{ 0 };
        WorldData currentWorld;
    };
}


#endif //RTT_WORLD_HPP
