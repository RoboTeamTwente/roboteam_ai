#include <iostream>
#include <chrono>
#include <thread>
#include <SDL2/SDL.h>
#include <SDL2/SDL_joystick.h>
#include <memory>
#include "roboteam_proto/Publisher.h"
#include "manual/JoystickManager.h"

using namespace std::chrono;

namespace rtt {
namespace input {

// TODO Check if these can be non-static
std::mutex JoystickManager::runningLock;
std::mutex JoystickManager::activeLock;

JoystickManager::JoystickManager(){
    std::cout << "[JoystickManager] New JoystickManager" << std::endl;
    // Create publisher to send robot commands
//    pub = std::make_unique<proto::Publisher<proto::RobotCommand>>("tcp://127.0.0.1:5556");
}

/** Calls the initialization and starts the loop */
bool JoystickManager::run() {
    if(!init()){
        std::cout << "[JoystickManager][start] Could not initialize JoystickManager. Exiting.." << std::endl;
        return false;
    }

    runningLock.lock();
    running = true;
    runningLock.unlock();

    activate();
    loop();
    return true;
}

/** Activate the handling of events and ticking of JoystickHandlers */
void JoystickManager::activate(){
    std::lock_guard<std::mutex> lock(activeLock);
    active = true;
    std::cout << "[JoystickManager][activate]" << std::endl;
}

/** Deactivate the handling of events and ticking of JoystickHandlers */
void JoystickManager::deactivate() {
    std::lock_guard<std::mutex> lock(activeLock);
    active = false;
    std::cout << "[JoystickManager][deactivate]" << std::endl;
}

/** Stops the loop of JoystickManager */
void JoystickManager::stop(){
    std::lock_guard<std::mutex> lock(runningLock);
    running = false;
    deactivate();
    std::cout << "[JoystickManager][stop]" << std::endl;
}

/** Checks if the JoystickManager is still running */
bool JoystickManager::isRunning(){
    std::lock_guard<std::mutex> lock(runningLock);
    return running;
}

/** Checks if the JoystickManager is still active */
bool JoystickManager::isActive(){
    std::lock_guard<std::mutex> lock(activeLock);
    return active;
}

/** Inits SDL */
bool JoystickManager::init() {
    std::cout << "[JoystickManager][init] Initializing.." << std::endl;
    if (SDL_InitSubSystem(SDL_INIT_JOYSTICK)){
        std::cout << "[JoystickManager][init] SDL_INIT_JOYSTICK failed : " << SDL_GetError() << std::endl;
        return false;
    }
    SDL_JoystickEventState(SDL_ENABLE);
    return true;
}

/** Main loop */
void JoystickManager::loop() {
    std::cout << "[JoystickManager][loop] Starting loop" << std::endl;
    int iTicks = 0;
    int iEvents = 0;
    SDL_Event event;

    steady_clock::time_point tTickNext = steady_clock::now();

    while(isRunning()) {
        /* Set the time to the next tick */
        tTickNext += milliseconds(TICK_INTERVAL);

        /* If inactive, sleep for TICK_INTERVAL */
        if(!isActive()){
            usleep(TICK_INTERVAL * 1000);
            continue;
        }

        /* Tick all JoystickHandlers */
        tickJoystickHandlers();
        iTicks++;

        /* Calculate the milliseconds to the next tick */
        int msToNextTick = (int) duration_cast<milliseconds>(tTickNext - steady_clock::now()).count();
        /* Ensure that we're waiting for a positive amount of time, else SDL_WaitEventTimeout will halt */
        msToNextTick = std::max(msToNextTick, 1);

        // std::cout << "msToNextTick=" << msToNextTick << std::endl;
        /* Wait for an event or until it is time for the next tick */
        while (SDL_WaitEventTimeout(&event, msToNextTick)) {

            /* Handle the received event */
            handleEvent(event);
            iEvents++;

            /* PANIC BUTTON! STOP EVERYTHING! */
            if (joystickHandlers.at(event.jdevice.which)->getJoystickState().XBOX)
                std::terminate();

            /* Check if there is time for another event, of if it is time for the next tick */
            msToNextTick = (int) duration_cast<milliseconds>(tTickNext - steady_clock::now()).count();
            if (msToNextTick <= 0)
                break;
        }
    }
    std::cout << "[JoystickManager][loop] Exiting loop" << std::endl;
}

/** Sends commands from JoystickHandlers */
void JoystickManager::tickJoystickHandlers(){
    for (const auto &joystickHandler : joystickHandlers) {
        joystickHandler.second->tick();
        // pub->send("robotcommands", joystickHandler.second->getCommand().SerializeAsString());
    }
}

/** Handles events and forwards them if needed */
void JoystickManager::handleEvent(SDL_Event& event){
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
void JoystickManager::handleJoystickAdded(const SDL_Event& event) {
    std::cout << "[JoystickManager][handleJoystickAdded] Adding joystick " << event.jdevice.which << std::endl;

    SDL_Joystick* joystick = SDL_JoystickOpen(event.jdevice.which);
    if(!joystick){
        std::cout << "[JoystickManager][handleJoystickAdded] Error! Could not open joystick!" << std::endl;
        return;
    }

    int instanceId = SDL_JoystickInstanceID(joystick);
    auto handler = new JoystickHandler();
    joystickHandlers.insert({instanceId, handler});
    std::cout << "[JoystickManager][handleJoystickAdded] Added joystick with InstanceID " << instanceId << std::endl;
}

/** Takes an SDL_Event and deletes and removes the correct JoystickHandler from the map of JoystickHandlers */
void JoystickManager::handleJoystickRemoved(const SDL_Event& event){
    std::cout << "[JoystickManager][handleJoystickRemoved] Removing joystick " << event.jdevice.which << std::endl;
    delete joystickHandlers.at(event.jdevice.which);
    joystickHandlers.erase(event.jdevice.which);
    std::cout << "[JoystickManager][handleJoystickAdded] Removed joystick with InstanceID " << event.jdevice.which << std::endl;
}

} // namespace input
} // namespace rtt

void startJoystickManager(rtt::input::JoystickManager& manager){
    std::cout << "[startJoystickManager] Starting JoystickManager in separate thread" << std::endl;
    manager.run();
    std::cout << "[startJoystickManager] Exiting separate thread" << std::endl;
}

int main(int argc, char **argv) {
    std::cout << "[JoystickManager.cpp][Main] starting" << std::endl;
    rtt::input::JoystickManager manager;

    // Start manager in separate thread
    std::thread joyThread(startJoystickManager, std::ref(manager));

/** Code to test threading */
//    sleep(5);
//    std::cout << "[Main] deactivating" << std::endl;
//    manager.deactivate();
//    sleep(5);
//    std::cout << "[Main] activating" << std::endl;
//    manager.activate();
//    sleep(5);
//    std::cout << "[Main] deactivating" << std::endl;
//    manager.deactivate();
//    sleep(5);
//    std::cout << "[Main] stopping manager" << std::endl;
//    manager.stop();

    // Wait for manager to finish
    joyThread.join();
    std::cout << "[JoystickManager.cpp][Main] stopping" << std::endl;


    return 0;
}
