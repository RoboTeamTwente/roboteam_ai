//
// Created by john on 12/16/19.
//

#ifndef RTT_WORLD_HPP
#define RTT_WORLD_HPP
#include <vector>
#include "views/WorldDataView.hpp"
#include "roboteam_proto/RobotFeedback.pb.h"
#include "WorldData.hpp"
#include "RobotControllers.hpp"

namespace rtt::world_new {

/**
 * Structure that represents the world and history of the world.
 *
 * This structure operates under the assumption of an immutable state
 * Adjusting any value inside this world after construction may
 * result in undefined behavior. Any sort of mutation of already constructed
 * WorldData will break the entire system and introduce dataraces due to
 * a shared immutable state.
 *
 * WorldData is a snapshot of the current world as seen by the camera
 * Feedback is already applied to the robots when they are constructed
 * As soon as a new world is constructed, which should only be done
 * through updateWorld, the current world is pushed to history, given that
 * this is a Some variant. Using any other unconventional way to push
 * a world to history will result in undefined behavior.
 *
 * The entire system with World relies on immutability,
 * this is something that Rust and haskell enforce and is seen as
 * one of the only ways to keep a project datarace-free
 */
    class World {
        public:
        /**
         * Global singleton for World, scott-meyers style
         * @return A pointer to a static World
         */
        inline static World *instance() {
          static World worldInstance{&rtt::SETTINGS};
          return &worldInstance;
        }

        /**
         * Not copyable, movable or assignable, global state
         */
        World(World const &) = delete;
        World &operator=(World &) = delete;

        World(World &&) = delete;
        World &operator=(World &&) = delete;

        /**
         * Amount of ticks to store in history
         */
        constexpr static size_t HISTORY_SIZE = 20;

        /**
         * Constructs a World from settings
         * @param settings Settings to use to construct the world, will be stored
         *
         * Undefined behavior if the pointer to settings outlives the lifetime of the settings instance
         *
         * Usage of settings before construction of the applicationmanager will result in undefined
         * behavior due to uninitialized memory
         */
        explicit World(Settings *settings);

        /**
         * Updates feedback for a specific robot
         * @param robotId Robot id of robot cache to update
         * @param feedback Feedback to apply, do not use after passing
         *
         * Undefined behavior may occur if feedback is used after being passed to this function
         */
        void updateFeedback(uint8_t robotId, proto::RobotFeedback &feedback);

        /**
         * Updates the currentWorld
         * @param world World to construct currentWorld from
         */
        void updateWorld(proto::World &protoWorld);

        /**
         * Gets the current world
         * @return std::nullopt if there is no currentWorld, otherwise Some with the value
         */
        [[nodiscard]] std::optional<view::WorldDataView> getWorld() const noexcept;

        /**
         * Gets a certain world from history
         * @param ticksAgo Ticks ago to fetch from
         * @return Returns the world at index currentIndex - ticksAgo
         */
        [[nodiscard]] view::WorldDataView getHistoryWorld(size_t ticksAgo) const noexcept;

        /**
         * Gets the difference in time between the last tick and the current tick
         * @return this->getWorldData()->getTime() - this->getHistoryWorld(1)->getTime();
         * Time format is in whatever protobuf provides
         */
        [[nodiscard]] uint64_t getTimeDifference() const noexcept;

        /**
         * Gets robot controllers for a specific robot
         * @param id Id of the robot to get the controllers from
         * @return A constant reference to the robot controller struct
         */
        [[nodiscard]] robot::RobotControllers &getControllersForRobot(uint8_t id) noexcept;

        /**
         * Gets the history size
         * @return size_t The amount of elements in the history
         */
        [[nodiscard]] size_t getHistorySize() const noexcept;

        private:
        /**
         * Upates the tickCount, sets lastTick to now(), sets duration to
         * oldNow - now();
         */
        void updateTickTime() noexcept;

        /**
         * Sets the current world, also pushes the old currentWorld into history if this is Some
         * Usage of currentWorld after call will result in UB
         * @param currentWorld New currentWorld
         * @return Returns a reference to the new currentWorld
         */
        WorldData const &setWorld(WorldData &currentWorld) noexcept;

        /**
         * Pushes a world to history, takes ownership of world
         * @param world WorldData to be taken ownership of
         */
        void toHistory(WorldData &world) noexcept;

        /**
         * Pointer to GUI settings
         */
        Settings *settings;

        /**
         * Mutex used when constructing robots to prevent updating of updateMap without wanting it
         */
        std::mutex updateMutex;

        /**
         * Map used to update robots on construction
         * Done so an immutable state can be preserved
         */
        std::unordered_map<uint8_t, proto::RobotFeedback> updateMap;

        /**
         * History of the world, this is where old world data is pushed to
         */
        std::vector<rtt::world_new::WorldData> history;

        /**
         * Map that maps robot ID's to their robot controllers
         */
        std::unordered_map<uint8_t, robot::RobotControllers> robotControllers{};

        /**
         * Current index into the ringbuffer that's the world history
         * https://en.wikipedia.org/wiki/Circular_buffer
         */
        size_t currentIndex{0};

        /**
         * Current world
         *
         * None if no world has been constructed yet
         * Some if a world is valid
         */
        std::optional<WorldData> currentWorld;

        /**
         * Timestamp of the last tick
         */
        uint64_t lastTick;

        /**
         * Duration between ticks
         */
        uint64_t tickDuration{};
    };
} // namespace rtt::world

#endif //RTT_WORLD_HPP
