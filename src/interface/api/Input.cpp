//
// Created by mrlukasbos on 4-12-18.
//

#include "interface/api/Input.h"

namespace rtt::ai::interface {

// declare static variables
std::vector<Drawing> Input::drawings;
std::mutex Input::drawingMutex;
std::mutex Input::fpsMutex;

int Input::FPS;

const std::vector<Drawing> Input::getDrawings() {
    std::lock_guard<std::mutex> lock(drawingMutex);
    return drawings;
}

void Input::clearDrawings() {
    std::lock_guard<std::mutex> drawingLock(drawingMutex);
    drawings.clear();
}

Input::~Input() { clearDrawings(); }

int Input::getFps() {
    std::lock_guard<std::mutex> lock(fpsMutex);
    return FPS;
}

void Input::setFps(int fps) {
    std::lock_guard<std::mutex> lock(fpsMutex);
    FPS = fps;
}

Input::Input() { drawings.reserve(10000); }

}  // namespace rtt::ai::interface