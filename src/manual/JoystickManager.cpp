#include "manual/JoystickManager.h"

#include <roboteam_utils/Print.h>

#include "world/World.hpp"
#include "world/views/RobotView.hpp"

using namespace std::chrono;

namespace rtt::input {

JoystickManager::JoystickManager() = default;

/** Calls the initialization and starts the loop */
bool JoystickManager::run() {
    if (!init()) {
        RTT_ERROR("Could not initialize JoystickManager. Exiting..")
        return false;
    }

    runningLock.lock();
    running = true;
    runningLock.unlock();

    loop();
    return true;
}

/** Activate the handling of events and ticking of JoystickHandlers */
void JoystickManager::activate() {
    std::lock_guard<std::mutex> lock(activeLock);
    active = true;
    RTT_INFO("JoystickManager is now active")
}

/** Deactivate the handling of events and ticking of JoystickHandlers */
void JoystickManager::deactivate() {
    std::lock_guard<std::mutex> lock(activeLock);
    active = false;
    RTT_INFO("JoystickManager is now inactive")
}

/** Stops the loop of JoystickManager */
void JoystickManager::stop() {
    std::lock_guard<std::mutex> lock(runningLock);
    running = false;
    deactivate();
    RTT_INFO("JoystickManager is now stopped")
}

/** Checks if the JoystickManager is still running */
bool JoystickManager::isRunning() {
    std::lock_guard<std::mutex> lock(runningLock);
    return running;
}

/** Checks if the JoystickManager is still active */
bool JoystickManager::isActive() {
    std::lock_guard<std::mutex> lock(activeLock);
    return active;
}

/** Inits SDL */
bool JoystickManager::init() {
    if (SDL_InitSubSystem(SDL_INIT_JOYSTICK)) {
        RTT_ERROR("SDL_INIT_JOYSTICK failed with error: " + std::string(SDL_GetError()))
        return false;
    }
    SDL_JoystickEventState(SDL_ENABLE);
    return true;
}

/** Main loop */
void JoystickManager::loop() {
    int iTicks = 0;
    int iEvents = 0;
    SDL_Event event;

    steady_clock::time_point tTickNext = steady_clock::now();

    while (isRunning()) {
        /* Set the time to the next tick */
        tTickNext += milliseconds(TICK_INTERVAL);

        /* If inactive, sleep for TICK_INTERVAL */
        if (!isActive()) {
            usleep(TICK_INTERVAL * 1000);
            continue;
        }

        /* Tick all JoystickHandlers */
        tickJoystickHandlers();
        iTicks++;

        /* Calculate the milliseconds to the next tick */
        int msToNextTick = (int)duration_cast<milliseconds>(tTickNext - steady_clock::now()).count();
        /* Ensure that we're waiting for a positive amount of time, else SDL_WaitEventTimeout will halt */
        msToNextTick = std::max(msToNextTick, 1);

        /* Wait for an event or until it is time for the next tick */
        while (SDL_WaitEventTimeout(&event, msToNextTick)) {
            /* Handle the received event */
            handleEvent(event);
            iEvents++;

            /* PANIC BUTTON! STOP EVERYTHING! */
            // Prevent std::out_of_range in joystickHandlers when handler was removed in handleEvent()
            if (joystickHandlers.count(event.jdevice.which) == 1)
                if (joystickHandlers.at(event.jdevice.which)->getJoystickState().XBOX) std::terminate();

            /* Check if there is time for another event, of if it is time for the next tick */
            msToNextTick = (int)duration_cast<milliseconds>(tTickNext - steady_clock::now()).count();
            if (msToNextTick <= 0) break;
        }
    }
    RTT_INFO("Exiting loop")
}

/** Sends commands from JoystickHandlers */
void JoystickManager::tickJoystickHandlers() {
    for (const auto &joystickHandler : joystickHandlers) {
        joystickHandler.second->tick();
        auto const &[_, world] = world::World::instance();
        auto robot = world->getWorld()->getRobotForId(joystickHandler.second->getCommand().id);
        if (joystickHandler.second->getCommand().kickSpeed > 0) {
            RTT_DEBUG("KickSpeed = ", joystickHandler.second->getCommand().kickSpeed);
            RTT_DEBUG("KickType = ", joystickHandler.second->getCommand().kickType == KickType::KICK     ? "Kick"
                                     : (joystickHandler.second->getCommand().kickType == KickType::CHIP) ? "Chip"
                                                                                                         : "NoKick");
        }
        rtt::ai::control::ControlModule::addRobotCommand(robot, joystickHandler.second->getCommand(), world);
    }
}

/** Handles events and forwards them if needed */
void JoystickManager::handleEvent(SDL_Event &event) {
    /* Catch device events if needed, else forward event to corresponding JoystickHandler */
    switch (event.type) {
        case SDL_JOYDEVICEADDED:
            handleJoystickAdded(event);
            break;
        case SDL_JOYDEVICEREMOVED:
            handleJoystickRemoved(event);
            break;
        default:
            joystickHandlers.at(event.jdevice.which)->handleEvent(event);
            break;
    }
}

/** Takes an SDL_Event and adds a new JoystickHandler to the map of JoystickHandlers */
void JoystickManager::handleJoystickAdded(const SDL_Event &event) {
    RTT_INFO("Adding joystick ", event.jdevice.which)

    SDL_Joystick *joystick = SDL_JoystickOpen(event.jdevice.which);
    if (!joystick) {
        RTT_ERROR("Could not open joystick ", event.jdevice.which)
        return;
    }

    int instanceId = SDL_JoystickInstanceID(joystick);
    auto handler = new JoystickHandler();
    joystickHandlers.insert({instanceId, handler});
    RTT_SUCCESS("Added joystick with InstanceID ", instanceId)
}

/** Takes an SDL_Event and deletes and removes the correct JoystickHandler from the map of JoystickHandlers */
void JoystickManager::handleJoystickRemoved(const SDL_Event &event) {
    RTT_INFO("Removing joystick with InstanceID ", event.jdevice.which)
    delete joystickHandlers.at(event.jdevice.which);
    joystickHandlers.erase(event.jdevice.which);
    RTT_SUCCESS("Removed joystick with InstanceID ", event.jdevice.which)
}

}  // namespace rtt::input
